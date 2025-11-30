#!/usr/bin/env python3
"""
FEAGI micro:bit BLE Agent

Connects to micro:bit via Bluetooth Low Energy and bridges to FEAGI Core.
Receives neuron firing data from FEAGI and sends it to micro:bit LED matrix.
"""

import asyncio
import json
import logging
from typing import Optional, Dict, Any
import sys
import os

# Add feagi_connector to path if not installed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../feagi_connector'))

try:
    from feagi_connector import FeagiClient, setup_agent_logging
except ImportError:
    print("ERROR: feagi_connector not found. Install it or ensure it's in the path.")
    sys.exit(1)

try:
    from bleak import BleakClient, BleakScanner
    from bleak.backends.characteristic import BleakGATTCharacteristic
except ImportError:
    print("ERROR: bleak not installed. Install with: pip install bleak")
    sys.exit(1)

# BLE Service UUIDs (from firmware bluetooth.rs)
FEAGI_SERVICE_UUID = "e95d0753-251d-470a-a062-fa1922dfa9a8"
NEURON_DATA_CHAR_UUID = "e95d0755-251d-470a-a062-fa1922dfa9a8"  # Write characteristic for neuron data
SENSOR_DATA_CHAR_UUID = "e95d0754-251d-470a-a062-fa1922dfa9a8"  # Notify characteristic for sensor data
CAPABILITIES_CHAR_UUID = "e95d0758-251d-470a-a062-fa1922dfa9a8"  # Read characteristic

# Device name (from firmware config)
MICROBIT_NAME = "FEAGI-microbit"  # Update this to match BLUETOOTH_NAME in firmware

logger = logging.getLogger(__name__)


class MicrobitBleAgent:
    """Agent that bridges FEAGI Core to micro:bit via BLE."""
    
    def __init__(self, feagi_host: str = "localhost", agent_id: str = "microbit-agent"):
        self.feagi_host = feagi_host
        self.agent_id = agent_id
        self.feagi_client: Optional[FeagiClient] = None
        self.ble_client: Optional[BleakClient] = None
        self.neuron_char: Optional[BleakGATTCharacteristic] = None
        self.connected = False
        
    async def connect_ble(self, device_name: str = MICROBIT_NAME) -> bool:
        """Connect to micro:bit via BLE."""
        logger.info(f"🔍 Scanning for micro:bit: {device_name}")
        
        # Scan for device
        devices = await BleakScanner.discover(timeout=10.0)
        target_device = None
        
        for device in devices:
            if device_name.lower() in device.name.lower() if device.name else False:
                target_device = device
                logger.info(f"✅ Found micro:bit: {device.name} ({device.address})")
                break
        
        if not target_device:
            logger.error(f"❌ micro:bit '{device_name}' not found")
            logger.info("Available devices:")
            for d in devices:
                logger.info(f"  - {d.name or 'Unknown'} ({d.address})")
            return False
        
        # Connect to device
        logger.info(f"🔌 Connecting to {target_device.address}...")
        self.ble_client = BleakClient(target_device)
        
        try:
            await self.ble_client.connect()
            logger.info("✅ Connected to micro:bit")
            
            # Find neuron data characteristic
            services = await self.ble_client.get_services()
            for service in services:
                if service.uuid.lower() == FEAGI_SERVICE_UUID.lower():
                    logger.info(f"✅ Found FEAGI service: {service.uuid}")
                    for char in service.characteristics:
                        logger.info(f"  Characteristic: {char.uuid} (properties: {char.properties})")
                        if char.uuid.lower() == NEURON_DATA_CHAR_UUID.lower() and "write" in char.properties:
                            self.neuron_char = char
                            logger.info(f"✅ Found neuron data characteristic: {char.uuid}")
                            break
            
            if not self.neuron_char:
                logger.warning("⚠️  Neuron data characteristic not found. Using Nordic UART Service as fallback.")
                # Fallback to NUS if FEAGI service not found
                for service in services:
                    if "6e400001" in service.uuid.lower():
                        for char in service.characteristics:
                            if "write" in char.properties:
                                self.neuron_char = char
                                logger.info(f"✅ Using NUS TX characteristic: {char.uuid}")
                                break
            
            if not self.neuron_char:
                logger.error("❌ No writable characteristic found")
                return False
            
            self.connected = True
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to connect: {e}")
            return False
    
    async def send_neuron_firing(self, coordinates: list):
        """
        Send neuron firing data to micro:bit.
        
        Packet format (from firmware bluetooth.rs):
        [0x01] [count] [x1, y1, x2, y2, ...]
        """
        if not self.connected or not self.neuron_char:
            logger.warning("⚠️  Not connected to micro:bit")
            return
        
        # Limit to 25 neurons (5x5 matrix)
        coords = coordinates[:25]
        count = len(coords)
        
        # Build packet: [command=0x01] [count] [x1, y1, x2, y2, ...]
        packet = bytearray([0x01, count])
        for x, y in coords:
            packet.extend([x, y])
        
        try:
            await self.ble_client.write_gatt_char(self.neuron_char, bytes(packet))
            logger.debug(f"📤 Sent {count} neuron coordinates to micro:bit")
        except Exception as e:
            logger.error(f"❌ Failed to send neuron data: {e}")
    
    def handle_motor_data(self, channel_id: str, data: bytes):
        """Handle motor data from FEAGI (neuron firing)."""
        try:
            # Parse JSON motor data from FEAGI
            motor_json = json.loads(data)
            
            # Extract cortical areas
            cortical_areas = motor_json.get("cortical_areas", {})
            
            # Look for LED Matrix cortical area (omis type)
            # Expected name: "LED Matrix" or "Display Matrix" or similar
            led_matrix_area = None
            for area_id, area_data in cortical_areas.items():
                # Check if this is the LED matrix area (you may need to adjust the name)
                if "led" in area_id.lower() or "matrix" in area_id.lower() or "display" in area_id.lower():
                    led_matrix_area = area_data
                    break
            
            if not led_matrix_area:
                # If no specific area found, use first area (for testing)
                if cortical_areas:
                    led_matrix_area = list(cortical_areas.values())[0]
                    logger.debug(f"Using first cortical area: {list(cortical_areas.keys())[0]}")
            
            if not led_matrix_area:
                return
            
            # Extract neuron coordinates
            # Format: {"neuron_ids": [...], "x": [...], "y": [...], "z": [...], "power": [...]}
            xs = led_matrix_area.get("x", [])
            ys = led_matrix_area.get("y", [])
            
            if not xs or not ys:
                return
            
            # Convert to list of (x, y) tuples, filtering to 5x5 range
            coordinates = []
            for x, y in zip(xs, ys):
                if 0 <= x < 5 and 0 <= y < 5:
                    coordinates.append((x, y))
            
            if coordinates:
                # Send to micro:bit asynchronously
                asyncio.create_task(self.send_neuron_firing(coordinates))
                logger.info(f"🎯 Sending {len(coordinates)} neuron firings to micro:bit LED matrix")
            
        except json.JSONDecodeError as e:
            logger.error(f"❌ Failed to parse motor data: {e}")
        except Exception as e:
            logger.error(f"❌ Error handling motor data: {e}")
    
    async def connect_feagi(self):
        """Connect to FEAGI Core."""
        logger.info(f"🔌 Connecting to FEAGI at {self.feagi_host}...")
        
        self.feagi_client = FeagiClient(
            host=self.feagi_host,
            agent_id=self.agent_id
        )
        
        try:
            await self.feagi_client.connect()
            
            # Register motor callback for LED matrix control
            self.feagi_client.register_motor_callback(self.handle_motor_data)
            
            logger.info("✅ Connected to FEAGI Core")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to connect to FEAGI: {e}")
            return False
    
    async def run(self):
        """Main run loop."""
        # Connect to micro:bit
        if not await self.connect_ble():
            return
        
        # Connect to FEAGI
        if not await self.connect_feagi():
            return
        
        logger.info("🚀 Agent running. Waiting for neuron firing data from FEAGI...")
        logger.info("💡 Make sure FEAGI has a cortical area mapped to LED Matrix (omis type, 5x5x1)")
        
        # Keep running
        try:
            while True:
                await asyncio.sleep(1.0)
        except KeyboardInterrupt:
            logger.info("🛑 Shutting down...")
        finally:
            if self.ble_client:
                await self.ble_client.disconnect()
            if self.feagi_client:
                await self.feagi_client.disconnect()


async def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="FEAGI micro:bit BLE Agent")
    parser.add_argument("--feagi-host", default="localhost", help="FEAGI Core host")
    parser.add_argument("--agent-id", default="microbit-agent", help="Agent ID")
    parser.add_argument("--device-name", default=MICROBIT_NAME, help="micro:bit device name")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose logging")
    
    args = parser.parse_args()
    
    # Setup logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    setup_agent_logging(log_level)
    
    # Create and run agent
    agent = MicrobitBleAgent(
        feagi_host=args.feagi_host,
        agent_id=args.agent_id
    )
    
    await agent.run()


if __name__ == "__main__":
    asyncio.run(main())


