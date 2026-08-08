"""MuJoCo viewer window geometry — placement and persistence for feagi-desktop.

On macOS we own the mjpython window, so we use NSWindow/CoreGraphics directly
via ctypes.  No Accessibility, no Automation, no osascript subprocess.

    • SET position  →  NSWindow.setFrameTopLeftPoint: + setContentSize:
                       dispatched to the main Cocoa thread via GCD dispatch_sync_f
    • GET position  →  CGWindowListCopyWindowInfo (no-permission read API)

On Linux we call GLFW directly (safe from the simulation thread there).
"""

from __future__ import annotations

import ctypes
import ctypes.util
import json
import logging
import os
import sys
import threading
import time
from typing import Any, Optional, Tuple

logger = logging.getLogger("mujoco_controller.viewer_geometry")

Rect = Tuple[int, int, int, int]  # x, y, width, height  (top-left origin)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _is_macos() -> bool:
    return sys.platform == "darwin"


def _geometry_file_path() -> Optional[str]:
    path = os.environ.get("FEAGI_CONTROLLER_GEOMETRY_FILE", "").strip()
    return path or None


def _target_rect_from_env() -> Optional[Rect]:
    """Read feagi-desktop launch target from env vars (logical pixels, top-left origin)."""
    keys = ("FEAGI_WINDOW_X", "FEAGI_WINDOW_Y", "FEAGI_WINDOW_WIDTH", "FEAGI_WINDOW_HEIGHT")
    vals: dict[str, str] = {}
    for k in keys:
        v = os.environ.get(k, "").strip()
        if not v:
            return None
        vals[k] = v
    try:
        x = int(vals["FEAGI_WINDOW_X"])
        y = int(vals["FEAGI_WINDOW_Y"])
        w = int(vals["FEAGI_WINDOW_WIDTH"])
        h = int(vals["FEAGI_WINDOW_HEIGHT"])
    except ValueError:
        return None
    return (x, y, w, h) if w > 0 and h > 0 else None


def _write_geometry_file(rect: Rect) -> None:
    """Atomic write of current geometry for the feagi-desktop Rust poller."""
    path = _geometry_file_path()
    if path is None:
        return
    x, y, w, h = rect
    payload = {"x": x, "y": y, "width": w, "height": h, "recorded_at_unix_s": time.time()}
    try:
        tmp = f"{path}.tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(payload, fh)
        os.replace(tmp, path)
    except OSError as exc:
        logger.debug("geometry file write failed: %s", exc)


# ---------------------------------------------------------------------------
# macOS — NSWindow via ctypes (no permissions; we own the window)
# ---------------------------------------------------------------------------

class _NSPoint(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double)]


class _NSSize(ctypes.Structure):
    _fields_ = [("width", ctypes.c_double), ("height", ctypes.c_double)]


class _NSRect(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double),
                ("width", ctypes.c_double), ("height", ctypes.c_double)]


def _macos_objc_setup():
    """Return configured libobjc, or None on failure."""
    try:
        lib = ctypes.CDLL(ctypes.util.find_library("objc"))
        lib.objc_getClass.restype = ctypes.c_void_p
        lib.sel_registerName.restype = ctypes.c_void_p
        return lib
    except Exception:
        return None


def _macos_get_screen_height(libobjc) -> Optional[float]:
    """Return logical height of the primary NSScreen (bottom-left Cocoa coords).
    
    We must use the primary screen (index 0) because CoreGraphics and Cocoa
    global coordinate systems are both anchored to the primary screen.
    """
    try:
        send = libobjc.objc_msgSend
        send.restype = ctypes.c_void_p
        send.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        
        NSScreen = libobjc.objc_getClass(b"NSScreen")
        screens = send(NSScreen, libobjc.sel_registerName(b"screens"))
        
        send.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
        primary_screen = send(screens, libobjc.sel_registerName(b"objectAtIndex:"), ctypes.c_ulong(0))

        send.restype = _NSRect
        send.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        rect = send(primary_screen, libobjc.sel_registerName(b"frame"))
        return float(rect.height)
    except Exception:
        return None


def _macos_get_window(libobjc) -> Optional[int]:
    """Return the largest NSWindow owned by this process."""
    try:
        send = libobjc.objc_msgSend

        # [NSApplication sharedApplication]
        send.restype = ctypes.c_void_p
        send.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        NSApp = libobjc.objc_getClass(b"NSApplication")
        app = send(NSApp, libobjc.sel_registerName(b"sharedApplication"))

        # [app windows]
        windows = send(app, libobjc.sel_registerName(b"windows"))

        # count
        send.restype = ctypes.c_ulong
        count = send(windows, libobjc.sel_registerName(b"count"))
        if not count:
            return None

        best_win = None
        best_area = 0
        sel_obj = libobjc.sel_registerName(b"objectAtIndex:")
        sel_frame = libobjc.sel_registerName(b"frame")

        for i in range(count):
            send.restype = ctypes.c_void_p
            send.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
            win = send(windows, sel_obj, ctypes.c_ulong(i))
            if not win:
                continue
            send.restype = _NSRect
            send.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
            f = send(win, sel_frame)
            area = int(f.width) * int(f.height)
            if area > best_area:
                best_area = area
                best_win = win

        return best_win
    except Exception:
        return None


def _macos_is_main_thread() -> bool:
    """Return True when called from the OS/Cocoa main thread (pthread_main_np)."""
    try:
        lib = ctypes.CDLL("/usr/lib/system/libsystem_pthread.dylib")
        lib.pthread_main_np.restype = ctypes.c_int
        lib.pthread_main_np.argtypes = []
        return bool(lib.pthread_main_np())
    except Exception:
        return False


def _macos_apply_window_rect(libobjc: Any, x: int, y: int, w: int, h: int) -> bool:
    """Apply NSWindow geometry — MUST be called from the Cocoa main thread."""
    screen_h = _macos_get_screen_height(libobjc)
    if screen_h is None:
        return False
    win = _macos_get_window(libobjc)
    if not win:
        return False

    send = libobjc.objc_msgSend

    # Cocoa uses bottom-left origin.
    # FEAGI provides (x, y) as top-left and (w, h) as frame size.
    # The bottom-left Y in Cocoa is screen_h - (y + h).
    cocoa_y = screen_h - float(y) - float(h)
    
    send.restype = None
    send.argtypes = [ctypes.c_void_p, ctypes.c_void_p, _NSRect, ctypes.c_bool]
    send(win, libobjc.sel_registerName(b"setFrame:display:"),
         _NSRect(float(x), cocoa_y, float(w), float(h)), True)

    return True


def _macos_do_set(x: int, y: int, w: int, h: int) -> bool:
    """Set NSWindow geometry, ensuring the call runs on the Cocoa main thread.

    NSWindow frame methods are NOT thread-safe on macOS — calling them from a
    background thread causes an AppKit exception and SIGABRT (EXC_CRASH).

    Strategy:
    • If already on the main thread → call _macos_apply_window_rect directly.
    • Otherwise → dispatch synchronously to the main queue via GCD
      dispatch_sync_f so the work executes on Thread 0 (the GLFW/Cocoa thread).

    GIL safety: ctypes releases the GIL when entering C, so the main-thread
    callback can re-acquire it for the Python closure.  No deadlock.
    """
    libobjc = _macos_objc_setup()
    if libobjc is None:
        return False

    if _macos_is_main_thread():
        # Already on main thread (e.g. tests, or future single-threaded launch).
        try:
            return _macos_apply_window_rect(libobjc, x, y, w, h)
        except Exception as exc:
            logger.debug("NSWindow set (direct) failed: %s", exc)
            return False

    # Background thread path: dispatch to main thread via GCD.
    try:
        libdispatch = ctypes.CDLL("/usr/lib/system/libdispatch.dylib")
        # _dispatch_main_q is the queue struct; its address IS the queue handle.
        _mq = (ctypes.c_byte * 1).in_dll(libdispatch, "_dispatch_main_q")
        main_queue = ctypes.c_void_p(ctypes.addressof(_mq))
    except Exception as exc:
        logger.debug("dispatch_get_main_queue unavailable: %s", exc)
        return False

    result: list[bool] = [False]
    WorkFn = ctypes.CFUNCTYPE(None, ctypes.c_void_p)

    def _work_on_main(_ctx: ctypes.c_void_p) -> None:  # runs on Thread 0
        try:
            result[0] = _macos_apply_window_rect(libobjc, x, y, w, h)
        except Exception as exc:
            logger.debug("NSWindow set (dispatched) failed: %s", exc)

    work_fn = WorkFn(_work_on_main)
    try:
        libdispatch.dispatch_sync_f.restype = None
        libdispatch.dispatch_sync_f.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, WorkFn
        ]
        libdispatch.dispatch_sync_f(main_queue, None, work_fn)
    except Exception as exc:
        logger.debug("dispatch_sync_f failed: %s", exc)
        return False

    return result[0]


def _macos_do_get() -> Optional[Rect]:
    """Read window bounds via CGWindowListCopyWindowInfo (no permissions required)."""
    try:
        cg = ctypes.CDLL(
            "/System/Library/Frameworks/CoreGraphics.framework/CoreGraphics"
        )
        cf = ctypes.CDLL(
            "/System/Library/Frameworks/CoreFoundation.framework/CoreFoundation"
        )

        # CGWindowListCopyWindowInfo(kCGWindowListOptionAll=0, kCGNullWindowID=0)
        cg.CGWindowListCopyWindowInfo.restype = ctypes.c_void_p
        cg.CGWindowListCopyWindowInfo.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
        arr = cg.CGWindowListCopyWindowInfo(0, 0)
        if not arr:
            return None

        cf.CFArrayGetCount.restype = ctypes.c_long
        cf.CFArrayGetCount.argtypes = [ctypes.c_void_p]
        cf.CFArrayGetValueAtIndex.restype = ctypes.c_void_p
        cf.CFArrayGetValueAtIndex.argtypes = [ctypes.c_void_p, ctypes.c_long]
        cf.CFDictionaryGetValue.restype = ctypes.c_void_p
        cf.CFDictionaryGetValue.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        cf.CFStringCreateWithCString.restype = ctypes.c_void_p
        cf.CFStringCreateWithCString.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32]
        cf.CFNumberGetValue.restype = ctypes.c_bool
        cf.CFNumberGetValue.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
        cf.CFRelease.restype = None
        cf.CFRelease.argtypes = [ctypes.c_void_p]

        def _cfstr(s: str) -> ctypes.c_void_p:
            return cf.CFStringCreateWithCString(None, s.encode(), 0x08000100)

        # kCFNumberIntType = 9 (for int); kCFNumberDoubleType = 13
        kCFNumberDoubleType = 13

        own_pid = os.getpid()
        key_pid = _cfstr("kCGWindowOwnerPID")
        key_bounds = _cfstr("kCGWindowBounds")
        key_x = _cfstr("X")
        key_y = _cfstr("Y")
        key_w = _cfstr("Width")
        key_h = _cfstr("Height")

        n = cf.CFArrayGetCount(arr)
        best: Optional[Rect] = None
        best_area = 0

        for i in range(n):
            entry = cf.CFArrayGetValueAtIndex(arr, i)
            if not entry:
                continue

            pid_val = cf.CFDictionaryGetValue(entry, key_pid)
            if not pid_val:
                continue
            pid_n = ctypes.c_int(0)
            cf.CFNumberGetValue(pid_val, 9, ctypes.byref(pid_n))  # kCFNumberIntType=9
            if pid_n.value != own_pid:
                continue

            bounds = cf.CFDictionaryGetValue(entry, key_bounds)
            if not bounds:
                continue

            def _read_double(d, k) -> Optional[float]:
                v = cf.CFDictionaryGetValue(d, k)
                if not v:
                    return None
                out = ctypes.c_double(0.0)
                cf.CFNumberGetValue(v, kCFNumberDoubleType, ctypes.byref(out))
                return out.value

            bx = _read_double(bounds, key_x)
            by = _read_double(bounds, key_y)
            bw = _read_double(bounds, key_w)
            bh = _read_double(bounds, key_h)
            if None in (bx, by, bw, bh):
                continue
            area = int(bw) * int(bh)
            if area > best_area:
                best_area = area
                best = (int(bx), int(by), int(bw), int(bh))

        for k in (key_pid, key_bounds, key_x, key_y, key_w, key_h):
            cf.CFRelease(k)
        cf.CFRelease(arr)

        return best
    except Exception as exc:
        logger.debug("CGWindowListCopyWindowInfo failed: %s", exc)
        return None


# ---------------------------------------------------------------------------
# Linux — GLFW (safe from the sim thread on Linux)
# ---------------------------------------------------------------------------

def _glfw_get_rect(viewer: Any) -> Optional[Rect]:
    try:
        import glfw  # type: ignore[import-untyped]
        with viewer.lock():
            win = glfw.get_current_context()
            if not win:
                return None
            px, py = glfw.get_window_pos(win)
            pw, ph = glfw.get_window_size(win)
        return (int(px), int(py), int(pw), int(ph)) if pw > 0 and ph > 0 else None
    except Exception as exc:
        logger.debug("GLFW get failed: %s", exc)
        return None


def _glfw_set_rect(viewer: Any, rect: Rect) -> bool:
    x, y, w, h = rect
    try:
        import glfw  # type: ignore[import-untyped]
        with viewer.lock():
            win = glfw.get_current_context()
            if not win:
                return False
            glfw.set_window_pos(win, x, y)
            glfw.set_window_size(win, w, h)
        return True
    except Exception as exc:
        logger.debug("GLFW set failed: %s", exc)
        return False


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def _macos_set_rect_with_retry(rect: Rect) -> None:
    """Retry-loop that waits for the window to appear, then positions it."""
    for attempt in range(1, 20):
        time.sleep(0.5)
        if _macos_do_set(*rect):
            logger.info(
                "[VIEW] Viewer geometry set: %dx%d at (%d, %d)",
                rect[2], rect[3], rect[0], rect[1],
            )
            return
    logger.warning("[VIEW] Could not set viewer geometry after retries.")


def apply_initial_geometry(viewer: Any) -> None:
    """Apply the feagi-desktop target rect to the viewer window.

    macOS: uses NSWindow via ctypes (no permissions — we own the window).
           Runs in a daemon thread so the simulation loop is not blocked.
    Linux: calls GLFW directly (safe from the simulation thread).
    """
    target = _target_rect_from_env()
    if target is None:
        return

    if _is_macos():
        t = threading.Thread(
            target=_macos_set_rect_with_retry,
            args=(target,),
            daemon=True,
            name="mujoco-geom-apply",
        )
        t.start()
    else:
        if _glfw_set_rect(viewer, target):
            logger.info(
                "[VIEW] Applied viewer geometry: %dx%d at (%d, %d)",
                target[2], target[3], target[0], target[1],
            )


def persist_geometry(viewer: Any) -> None:
    """Snapshot current viewer geometry to the feagi-desktop geometry file."""
    rect: Optional[Rect] = None
    if _is_macos():
        rect = _macos_do_get()
    else:
        rect = _glfw_get_rect(viewer)
    if rect is not None:
        _write_geometry_file(rect)


class ViewerGeometryRecorder:
    """Rate-limited geometry persistence tied to the simulation loop."""

    def __init__(self, interval_s: float = 2.0) -> None:
        self._interval_s = interval_s
        self._last: float = 0.0

    def tick(self, viewer: Any) -> None:
        now = time.monotonic()
        if now - self._last < self._interval_s:
            return
        self._last = now
        persist_geometry(viewer)

    def flush(self, viewer: Any) -> None:
        persist_geometry(viewer)
