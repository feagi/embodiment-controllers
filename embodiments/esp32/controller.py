#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""FEAGI ESP32 LED embodiment controller.

FEAGI -> this controller -> USB serial -> ESP32 (firmware/esp32_led) -> LED

The LED is driven by a one-device PositionalServo (``opse``) output area, a
1x1x10 column of neurons. The controller creates the area if the loaded genome
does not have it.

  * any neuron in the column fires   -> LED on. Brightness comes from the Z
                                        level that fired: z=0 -> 100 %, higher z
                                        is dimmer; the exact scale depends on the
                                        FEAGI version
  * nothing fires for ``off_after_s`` -> LED off

FEAGI's absolute opse decoder holds its last value, so the "no activity -> off"
gate lives here. That makes the LED follow brain activity: a genome that pulses
the area makes the LED blink (see genome/esp32_led_blink.genome).

The controller also watches FEAGI's health endpoint and re-registers on its own
when FEAGI loads a new genome or restarts.

Dependencies: feagi (Apache-2.0), pyserial (BSD-3-Clause).

Copyright 2026 Neuraville Inc.
"""

from __future__ import annotations

import argparse
import base64
import json
import logging
import os
import struct
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

HERE = Path(__file__).resolve().parent

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                    datefmt="%H:%M:%S")
logger = logging.getLogger("esp32_led")


def make_descriptor_b64(manufacturer: str, name: str, version: int, instance: int = 1) -> str:
    """48-byte FEAGI agent descriptor: instance u32 + manufacturer[20] + name[20] + version u32."""
    m = manufacturer.encode("ascii")[:20].ljust(20, b"\x00")
    n = name.encode("ascii")[:20].ljust(20, b"\x00")
    raw = struct.pack("<I", instance) + m + n + struct.pack("<I", version)
    return base64.b64encode(raw).decode("ascii")


def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="FEAGI ESP32 LED controller")
    p.add_argument("--networking-config", default=str(HERE / "networking.json"))
    p.add_argument("--capabilities-config", default=str(HERE / "capabilities.json"))
    p.add_argument("--serial-port", help="ESP32 serial port, e.g. COM5 or /dev/ttyUSB0. "
                                         "Omit for a dry run (FEAGI only, no hardware).")
    p.add_argument("--baud", type=int)
    p.add_argument("--cortical-group-id", type=int)
    p.add_argument("--channel", type=int)
    p.add_argument("--off-after-s", type=float,
                   help="Seconds without firing before the LED turns off.")
    p.add_argument("--on-off", action="store_true", default=None,
                   help="Ignore brightness: any firing = 100%%.")
    p.add_argument("--feagi-host")
    p.add_argument("--feagi-api-port", type=int)
    p.add_argument("--feagi-registration-port", type=int)
    p.add_argument("--feagi-sensory-port", type=int)
    p.add_argument("--feagi-motor-port", type=int)
    p.add_argument("--auth-token-b64")
    p.add_argument("--agent-id")
    return p.parse_args()


def resolve_config(args: argparse.Namespace) -> Dict[str, Any]:
    """Merge networking.json + capabilities.json, with any --flag taking precedence."""
    net = load_json(Path(args.networking_config))
    caps = load_json(Path(args.capabilities_config))
    fs, ag = net["feagi_settings"], net["agent_settings"]
    led, ser = caps["led"], caps["serial"]

    def pick(flag_value, default):
        return default if flag_value is None else flag_value

    return {
        "feagi_host": pick(args.feagi_host, fs["feagi_host"]),
        "feagi_api_port": pick(args.feagi_api_port, fs["feagi_api_port"]),
        "feagi_registration_port": pick(args.feagi_registration_port, fs["feagi_registration_port"]),
        "feagi_sensory_port": pick(args.feagi_sensory_port, fs["feagi_sensory_port"]),
        "feagi_motor_port": pick(args.feagi_motor_port, fs["feagi_motor_port"]),
        "feagi_http_timeout_s": fs["feagi_http_timeout_s"],
        "heartbeat_interval_s": fs["heartbeat_interval_s"],
        "connection_timeout_ms": fs["connection_timeout_ms"],
        "registration_retries": fs["registration_retries"],
        "agent_id": pick(args.agent_id, ag["agent_id"]),
        "manufacturer": ag["agent_descriptor_manufacturer"],
        "descriptor_name": ag["agent_descriptor_name"],
        "descriptor_version": int(ag["agent_descriptor_version"]),
        "auth_token_b64": pick(args.auth_token_b64, ag["auth_token_b64"]),
        "group_id": int(pick(args.cortical_group_id, led["cortical_group_id"])),
        "channel": int(pick(args.channel, led["channel"])),
        "off_after_s": float(pick(args.off_after_s, led["off_after_s"])),
        "on_off": bool(pick(args.on_off, led["on_off"])),
        "serial_port": pick(args.serial_port, ser.get("port")),
        "baud": int(pick(args.baud, ser["baud"])),
        "loop_period_s": float(caps.get("loop_period_s", 0.02)),
    }


def open_serial(port: Optional[str], baud: int):
    if not port:
        logger.warning("No serial port set -> DRY RUN (talking to FEAGI only, no hardware).")
        return None
    try:
        import serial  # pyserial
        ser = serial.Serial(port, baud, timeout=0)
        time.sleep(2.0)  # opening the port resets most ESP32 boards; let the sketch boot
        ser.reset_input_buffer()
        logger.info("Opened %s @ %d baud.", port, baud)
        return ser
    except Exception as exc:  # noqa: BLE001 - fall back to dry run on any serial failure
        logger.error("Could not open %s (%s) -> DRY RUN.", port, exc)
        return None


def main() -> int:
    cfg = resolve_config(parse_args())
    base_url = f"http://{cfg['feagi_host']}:{cfg['feagi_api_port']}"

    # 1) Make sure the LED output area exists --------------------------------
    from cortical_area_provisioner import ensure_servo_cortical_area
    from feagi.genome.api import GenomeAPIError
    try:
        made = ensure_servo_cortical_area(base_url, 1, group_id=cfg["group_id"],
                                          timeout=cfg["feagi_http_timeout_s"])
        logger.info("LED output area (opse) %s.", "created" if made else "already present")
    except GenomeAPIError as exc:
        logger.error("FEAGI not reachable at %s: %s", base_url, exc)
        return 1

    # 2) Register the output. The descriptor env var must be set before
    #    register(), because it lazily initialises the motor cache.
    descriptor = make_descriptor_b64(cfg["manufacturer"], cfg["descriptor_name"],
                                     cfg["descriptor_version"])
    os.environ["FEAGI_AGENT_DESCRIPTOR_B64"] = descriptor
    from feagi.pns import brain_output
    from feagi.pns.health_monitor import FeagiHealthMonitor
    from feagi.pns.outputs import ServoMotor
    led = ServoMotor.register(range=(0, 100), encoding="absolute",
                              unit_id=cfg["group_id"], channel_index=cfg["channel"])

    # 3) Connect ---------------------------------------------------------------
    brain_output.configure(
        agent_id=descriptor,
        feagi_host=cfg["feagi_host"],
        feagi_registration_port=cfg["feagi_registration_port"],
        feagi_sensory_port=cfg["feagi_sensory_port"],
        feagi_motor_port=cfg["feagi_motor_port"],
        transport="zmq",
        feagi_connection_timeout_ms=cfg["connection_timeout_ms"],
        feagi_registration_retries=cfg["registration_retries"],
        feagi_heartbeat_interval_s=cfg["heartbeat_interval_s"],
        feagi_api_port=cfg["feagi_api_port"],
        feagi_http_timeout_s=cfg["feagi_http_timeout_s"],
        auth_token_b64=cfg["auth_token_b64"],
    )
    brain_output.connect()
    logger.info("Connected to FEAGI as '%s'. Fire the opse area to light the LED.", cfg["agent_id"])

    # Without this, loading a new genome silently stops the motor stream.
    monitor = FeagiHealthMonitor(
        feagi_api_host=cfg["feagi_host"],
        feagi_api_port=cfg["feagi_api_port"],
        fetch_timeout_ms=1000,
        cooldown_ms=2000,
        max_consecutive_failures=30,
        trigger_on_session_changed=True,
        trigger_on_genome_changed=True,
        trigger_on_genome_load_completed=True,
        trigger_on_back_online=True,
        trigger_on_brain_ready=False,
        trigger_on_transport_send_failed=True,
    )

    # 4) Main loop -------------------------------------------------------------
    ser = open_serial(cfg["serial_port"], cfg["baud"])
    last_seq = led._rx_command_seq
    last_fire = 0.0
    last_health = 0.0
    sent: Optional[int] = None
    try:
        while True:
            now = time.monotonic()
            if now - last_health >= 1.0:
                last_health = now
                try:
                    decision = monitor.tick(now_ms=int(now * 1000)).decision
                except Exception as exc:  # noqa: BLE001
                    logger.warning("Health check failed: %s", exc)
                    decision = None
                if decision is not None and decision.is_attempt_now():
                    logger.info("FEAGI changed (%s) -> reconnecting...", decision.reason)
                    try:
                        # A newly loaded genome may not have the LED area.
                        ensure_servo_cortical_area(base_url, 1, group_id=cfg["group_id"],
                                                   timeout=cfg["feagi_http_timeout_s"])
                        brain_output.reconnect(reason=decision.reason)
                        monitor.record_attempt_succeeded()
                        logger.info("Reconnected.")
                    except Exception as exc:  # noqa: BLE001
                        monitor.record_attempt_failed()
                        logger.warning("Reconnect failed: %s", exc)

            brain_output.receive()
            now = time.monotonic()
            if led._rx_command_seq != last_seq:  # FEAGI sent a value since last loop
                last_seq = led._rx_command_seq
                last_fire = now
            if (now - last_fire) < cfg["off_after_s"]:
                level = 100 if cfg["on_off"] else max(1, int(round(led.get_angle())))
            else:
                level = 0
            if level != sent:
                if ser is not None:
                    ser.write(f"{level}\n".encode("ascii"))
                logger.info("LED -> %d%%", level)
                sent = level
            if ser is not None and ser.in_waiting:
                for line in ser.read(ser.in_waiting).decode(errors="ignore").splitlines():
                    if line.strip():
                        logger.debug("esp32: %s", line.strip())
            time.sleep(cfg["loop_period_s"])
    except KeyboardInterrupt:
        logger.info("Stopping.")
    finally:
        if ser is not None:
            ser.write(b"0\n")
            ser.close()
        try:
            brain_output.disconnect()
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
