#!/usr/bin/env python3
"""FEAGI DAVIS346 event-camera embodiment controller.

Streams iniVation DAVIS346 event-camera output into FEAGI as XYZP voxels
in a simple-vision cortical area, either from a live USB device
(via `neuromorphic_drivers`) or from a recorded AEDAT4 file (via `aedat`).

Event cameras already encode temporal change; this controller injects raw
event voxels directly and does not run FEAGI's RGB frame-difference
pipeline.

All licensing-sensitive dependencies used here are permissively licensed:
- neuromorphic_drivers: MIT (transitively links libusb, LGPL-2.1, via rusb)
- aedat: MIT, no USB dependency, pure AEDAT4 file decoder
- feagi (feagi-python-sdk): Apache-2.0

Copyright 2026 Neuraville Inc.
"""

from __future__ import annotations

import argparse
import base64
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any, List, Optional, Tuple

import numpy as np

from controller_config import (
    DEFAULT_CAPABILITIES_FILENAME,
    DEFAULT_NETWORKING_FILENAME,
    apply_config_defaults,
    flatten_capabilities_config,
    flatten_networking_config,
    load_json_config,
)
from cortical_area_provisioner import ensure_simple_vision_cortical_area
from davis_events import (
    DAVIS346_HEIGHT,
    DAVIS346_WIDTH,
    events_to_xyzp,
    playback_delay_s,
    polarity_events_to_columns,
)

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [DAVIS346] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
    stream=sys.stderr,
)
logger = logging.getLogger(__name__)


def _require_neuromorphic_drivers() -> Any:
    """Import neuromorphic_drivers. Fail closed if it is not installed."""
    try:
        import neuromorphic_drivers as nd
    except ImportError as exc:
        raise RuntimeError(
            "neuromorphic_drivers is not installed. "
            "Install it with: pip install neuromorphic_drivers"
        ) from exc
    return nd


def _require_aedat() -> Any:
    """Import aedat. Fail closed if it is not installed."""
    try:
        import aedat
    except ImportError as exc:
        raise RuntimeError(
            "aedat is not installed. Install it with: pip install aedat"
        ) from exc
    return aedat


def _require_feagi_sdk() -> Tuple[Any, Any]:
    """Import the FEAGI Python SDK client and its Rust-backed data layer."""
    try:
        from feagi.pns.client import AgentType, FeagiAgentClient
    except ImportError as exc:
        raise RuntimeError(
            f"feagi-python-sdk is not installed in {sys.executable}. "
            "Install it with: pip install feagi"
        ) from exc
    try:
        import feagi_rust_py_libs as frpl
    except ImportError as exc:
        raise RuntimeError(
            f"feagi_rust_py_libs is not installed in {sys.executable}"
        ) from exc
    return FeagiAgentClient, AgentType, frpl


def _parse_coordinates_3d(value: str) -> Tuple[int, int, int]:
    """Parse a 'x,y,z' CLI value into a 3-tuple of ints."""
    parts = value.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"Expected 'x,y,z', got {value!r}")
    try:
        return (int(parts[0]), int(parts[1]), int(parts[2]))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Expected 'x,y,z' integers, got {value!r}") from exc


def run_discover() -> None:
    """Print connected DAVIS346 cameras as JSON on stdout."""
    nd = _require_neuromorphic_drivers()
    cameras = []
    for device in nd.list_devices():
        if device.name != nd.Name.INIVATION_DAVIS346:
            continue
        cameras.append(
            {
                "camera_model": "DAVIS346",
                "serial_number": device.serial,
                "camera_name": f"DAVIS346_{device.serial}",
                "width": DAVIS346_WIDTH,
                "height": DAVIS346_HEIGHT,
            }
        )
    sys.stdout.write(json.dumps({"cameras": cameras}))
    sys.stdout.flush()


def _build_agent_descriptor_b64(manufacturer: str, agent_name: str, agent_version: int) -> str:
    """Build the 48-byte AgentDescriptor payload used by the Python SDK."""
    if not manufacturer or not manufacturer.isascii() or len(manufacturer) > 20:
        raise ValueError("agent_descriptor_manufacturer must be ASCII and <= 20 chars")
    if not agent_name or not agent_name.isascii() or len(agent_name) > 20:
        raise ValueError("agent_descriptor_name must be ASCII and <= 20 chars")
    if agent_version <= 0:
        raise ValueError("agent_descriptor_version must be > 0")

    payload = bytearray(48)
    payload[0:4] = (0).to_bytes(4, byteorder="little", signed=False)
    payload[4:24] = manufacturer.encode("ascii").ljust(20, b"\x00")
    payload[24:44] = agent_name.encode("ascii").ljust(20, b"\x00")
    payload[44:48] = int(agent_version).to_bytes(4, byteorder="little", signed=False)
    return base64.b64encode(payload).decode("ascii")


def _ensure_cortical_area(args: argparse.Namespace, width: int, height: int) -> str:
    """Create or resize the target simple-vision cortical area over FEAGI's REST API."""
    from feagi.genome.api import GenomeAPI

    genome_api = GenomeAPI(
        base_url=f"http://{args.feagi_host}:{args.feagi_api_port}",
        timeout=args.feagi_http_timeout_s,
    )
    cortical_id = ensure_simple_vision_cortical_area(
        genome_api,
        unit_index=args.cortical_group_id,
        width=width,
        height=height,
        depth=2,
        coordinates_3d=args.cortical_coordinates_3d,
    )
    logger.info("Target simple-vision cortical area: %s", cortical_id)
    return cortical_id


def _connect_feagi_client(
    args: argparse.Namespace, cortical_id: str, width: int, height: int
) -> Any:
    """Build, configure, and connect a FeagiAgentClient sensory agent."""
    FeagiAgentClient, AgentType, _frpl = _require_feagi_sdk()

    agent_descriptor_b64 = _build_agent_descriptor_b64(
        manufacturer=args.agent_descriptor_manufacturer,
        agent_name=args.agent_descriptor_name,
        agent_version=args.agent_descriptor_version,
    )

    client = FeagiAgentClient(args.agent_id, AgentType.SENSORY)
    client.configure(
        feagi_host=args.feagi_host,
        registration_port=args.feagi_registration_port,
        sensory_port=args.feagi_sensory_port,
        motor_port=args.feagi_motor_port,
        agent_descriptor_b64=agent_descriptor_b64,
        auth_token_b64=args.auth_token_b64,
        vision_unit=("davis_events", width, height, 2, "iimg", args.cortical_group_id),
        heartbeat_interval=args.heartbeat_interval_s,
        connection_timeout_ms=args.connection_timeout_ms,
        registration_retries=args.registration_retries,
        feagi_api_port=args.feagi_api_port,
        feagi_http_timeout_s=args.feagi_http_timeout_s,
    )
    if not client.connect(graceful=True):
        raise RuntimeError(f"Failed to connect to FEAGI at {args.feagi_host}")
    logger.info("Connected to FEAGI; injecting XYZP into cortical area %s", cortical_id)
    return client


def _send_xyzp(
    client: Any,
    frpl: Any,
    cortical_id_b64: str,
    xs: np.ndarray,
    ys: np.ndarray,
    zs: np.ndarray,
    ps: np.ndarray,
) -> int:
    """Serialize one burst of XYZP voxels into a FeagiByteContainer and send it."""
    if xs.size == 0:
        return 0

    cortical_id = frpl.data_structures.genomic.cortical_area.CorticalID.try_from_base_64(
        cortical_id_b64
    )
    neuron_arrays = frpl.data_structures.neurons_voxels.xyzp.PyNeuronVoxelXYZPArrays.new_from_numpy(
        xs, ys, zs, ps
    )
    data = frpl.data_structures.neurons_voxels.xyzp.CorticalMappedXYZPNeuronVoxels()
    data.insert(cortical_id, neuron_arrays)
    container = frpl.data_serialization.FeagiByteContainer()
    container.add_struct(data, increment_value=0)
    client.send_sensory_bytes(bytes(container.copy_out_as_byte_vector()))
    return int(xs.size)


def _inject_polarity_events(
    client: Any,
    frpl: Any,
    cortical_id: str,
    polarity_events: np.ndarray,
    width: int,
    height: int,
    args: argparse.Namespace,
) -> int:
    """Map one polarity-events batch to XYZP and inject it into FEAGI."""
    xs, ys, polarities, _timestamps = polarity_events_to_columns(polarity_events)
    x_out, y_out, z_out, p_out = events_to_xyzp(
        xs, ys, polarities, width=width, height=height, potential=args.event_potential
    )
    return _send_xyzp(client, frpl, cortical_id, x_out, y_out, z_out, p_out)


def run_stream(args: argparse.Namespace) -> None:
    """Capture live DAVIS346 events and inject XYZP into simple vision."""
    logger.info(
        "Starting DAVIS346 live stream serial=%s", args.camera_serial or "(any)"
    )
    _FeagiAgentClient, _AgentType, frpl = _require_feagi_sdk()
    nd = _require_neuromorphic_drivers()

    configuration = nd.inivation_davis346.Configuration()
    device = nd.open(configuration=configuration, serial=args.camera_serial)
    try:
        properties = device.properties()
        width, height = int(properties.width), int(properties.height)
        logger.info(
            "Opened DAVIS346 serial=%s resolution=%sx%s", device.serial(), width, height
        )

        cortical_id = _ensure_cortical_area(args, width, height)
        client = _connect_feagi_client(args, cortical_id, width, height)
        sent_total = 0
        try:
            for status, packet in device:
                if packet is None:
                    continue
                if packet.polarity_events is not None and len(packet.polarity_events) > 0:
                    sent_total += _inject_polarity_events(
                        client, frpl, cortical_id, packet.polarity_events, width, height, args
                    )
        finally:
            logger.info("DAVIS346 stream stopping; voxels sent=%s", sent_total)
            client.disconnect()
    finally:
        device.close()


def run_playback(args: argparse.Namespace) -> None:
    """Play an AEDAT4 event recording into simple vision at recorded timing."""
    aedat_path = Path(args.aedat_path)
    if not aedat_path.is_file():
        raise FileNotFoundError(f"AEDAT4 recording not found: {aedat_path}")
    if aedat_path.suffix.lower() != ".aedat4":
        raise ValueError(f"DAVIS346 playback requires a .aedat4 file, got {aedat_path.name}")

    logger.info("Starting AEDAT4 playback path=%s loop=%s", aedat_path, args.loop)
    _FeagiAgentClient, _AgentType, frpl = _require_feagi_sdk()
    aedat = _require_aedat()

    decoder = aedat.Decoder(str(aedat_path))
    event_stream_id: Optional[int] = None
    width, height = DAVIS346_WIDTH, DAVIS346_HEIGHT
    for stream_id, stream in decoder.id_to_stream().items():
        if stream["type"] == "events":
            event_stream_id = stream_id
            width = int(stream.get("width", width))
            height = int(stream.get("height", height))
            break
    if event_stream_id is None:
        raise RuntimeError(f"AEDAT4 has no event stream: {aedat_path}")
    logger.info("AEDAT4 event stream id=%s resolution=%sx%s", event_stream_id, width, height)

    cortical_id = _ensure_cortical_area(args, width, height)
    client = _connect_feagi_client(args, cortical_id, width, height)
    sent_total = 0
    try:
        while True:
            origin_ts_us: Optional[int] = None
            origin_wall_s: Optional[float] = None
            for packet in decoder:
                if packet["stream_id"] != event_stream_id or "events" not in packet:
                    continue
                events = packet["events"]
                if len(events) == 0:
                    continue
                first_ts = int(events["t"][0])
                if origin_ts_us is None or origin_wall_s is None:
                    origin_ts_us = first_ts
                    origin_wall_s = time.time()
                else:
                    delay = playback_delay_s(first_ts, origin_ts_us, origin_wall_s, time.time())
                    if delay > 0.0:
                        time.sleep(delay)
                sent_total += _inject_polarity_events(
                    client, frpl, cortical_id, events, width, height, args
                )
            if not args.loop:
                break
            logger.info("AEDAT4 loop restart %s", aedat_path)
            decoder = aedat.Decoder(str(aedat_path))
    finally:
        logger.info("DAVIS346 playback stopping; voxels sent=%s", sent_total)
        client.disconnect()


def _feagi_required_args() -> List[str]:
    """CLI fields required to connect the agent to FEAGI."""
    return [
        "cortical_group_id",
        "cortical_coordinates_3d",
        "agent_id",
        "feagi_host",
        "feagi_api_port",
        "feagi_registration_port",
        "feagi_sensory_port",
        "feagi_motor_port",
        "feagi_http_timeout_s",
        "heartbeat_interval_s",
        "connection_timeout_ms",
        "registration_retries",
        "agent_descriptor_manufacturer",
        "agent_descriptor_name",
        "agent_descriptor_version",
        "auth_token_b64",
    ]


def main() -> int:
    """CLI entry point for DAVIS346 discovery, live capture, and AEDAT4 playback.

    All connection, agent, and cortical-targeting parameters are read from
    the bundled `networking.json` / `capabilities.json` next to this script
    by default, so a one-time edit of those files is enough for routine
    `--mode stream` / `--mode playback` invocations. Any CLI flag overrides
    its config-file counterpart for that run; nothing is hardcoded in code,
    and any field left unset by both config and CLI still fails loudly.
    """
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="FEAGI DAVIS346 event-camera controller")
    parser.add_argument("--mode", required=True, choices=("discover", "stream", "playback"))
    parser.add_argument(
        "--networking-config",
        default=str(script_dir / DEFAULT_NETWORKING_FILENAME),
        help="Path to networking.json (FEAGI host/ports/agent identity defaults)",
    )
    parser.add_argument(
        "--capabilities-config",
        default=str(script_dir / DEFAULT_CAPABILITIES_FILENAME),
        help="Path to capabilities.json (cortical target/camera/playback defaults)",
    )
    parser.add_argument("--camera-serial", help="DAVIS346 USB serial number (omit to open any)")
    parser.add_argument("--aedat-path", help="Path to an AEDAT4 DAVIS recording")
    parser.add_argument(
        "--loop",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Replay AEDAT4 when it ends (overrides capabilities.json)",
    )
    parser.add_argument("--cortical-group-id", type=int)
    parser.add_argument(
        "--cortical-coordinates-3d",
        type=_parse_coordinates_3d,
        help="Position 'x,y,z' for the cortical area if it must be created",
    )
    parser.add_argument("--event-potential", type=float)
    parser.add_argument("--agent-id")
    parser.add_argument("--feagi-host")
    parser.add_argument("--feagi-api-port", type=int)
    parser.add_argument("--feagi-registration-port", type=int)
    parser.add_argument("--feagi-sensory-port", type=int)
    parser.add_argument("--feagi-motor-port", type=int)
    parser.add_argument("--feagi-http-timeout-s", type=float)
    parser.add_argument("--heartbeat-interval-s", type=float)
    parser.add_argument("--connection-timeout-ms", type=int)
    parser.add_argument("--registration-retries", type=int)
    parser.add_argument("--agent-descriptor-manufacturer")
    parser.add_argument("--agent-descriptor-name")
    parser.add_argument("--agent-descriptor-version", type=int)
    parser.add_argument("--auth-token-b64")
    args = parser.parse_args()

    if args.mode == "discover":
        run_discover()
        return 0

    networking_defaults = flatten_networking_config(
        load_json_config(Path(args.networking_config))
    )
    capabilities_defaults = flatten_capabilities_config(
        load_json_config(Path(args.capabilities_config))
    )
    merged = apply_config_defaults(vars(args), networking_defaults)
    merged = apply_config_defaults(merged, capabilities_defaults)
    args = argparse.Namespace(**merged)

    required = _feagi_required_args()
    if args.mode == "playback":
        required.append("aedat_path")
    # camera_serial is optional in stream mode: omit to open the first matching device.
    missing = [name for name in required if getattr(args, name) in (None, "")]
    if missing:
        parser.error(
            f"{args.mode} mode requires (via CLI flag or config file): {', '.join(missing)}"
        )

    try:
        if args.mode == "stream":
            run_stream(args)
        else:
            run_playback(args)
    except Exception:
        logger.exception("DAVIS346 controller failed")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
