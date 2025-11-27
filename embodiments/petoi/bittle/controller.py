#!/usr/bin/env python
"""
Bittle X Bluetooth Controller

Simple controller for Petoi Bittle X using FEAGI Python SDK.
Demonstrates how easy it is to connect a Bluetooth robot!

Copyright 2016-2025 Neuraville Inc. All Rights Reserved.
"""

import asyncio
import argparse
from feagi.agent import BluetoothRobot


class BittleRobot(BluetoothRobot):
    """
    Petoi Bittle X quadruped robot controller.
    
    Only ~40 lines of robot-specific code!
    All BLE and FEAGI complexity handled by SDK.
    """
    
    # Bluetooth configuration (from embodiment.json)
    BLUETOOTH_CONFIG = {
        "device_name": "Bittle",
        "service_uuid": "6e400001-b5a3-f393-e0a9-e50e24dcca9e",
        "rx_uuid": "6e400002-b5a3-f393-e0a9-e50e24dcca9e",
        "tx_uuid": "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
    }
    
    # FEAGI-to-Bittle servo ID mapping
    SERVO_MAP = {
        0: 0,   # Head
        1: 8,   # Front left top
        2: 12,  # Front left bottom
        3: 9,   # Front right top
        4: 13,  # Front right bottom
        5: 11,  # Rear left top
        6: 15,  # Rear left bottom
        7: 10,  # Rear right top
        8: 14   # Rear right bottom
    }
    
    def parse_sensors(self, raw_bytes: bytes) -> dict:
        """
        Parse Bittle's sensor data.
        
        Bittle sends gyro data as: "x,y,z#"
        """
        try:
            data_str = raw_bytes.decode('utf-8').strip()
            
            # Handle gyro data (format: "x,y,z#")
            if '#' in data_str:
                # Clean and split
                data_str = data_str.replace('\r', '').replace('\n', '')
                values_str = data_str.split('#')[0]
                values = values_str.split(',')
                
                # Parse floats
                gyro_data = []
                for val in values:
                    clean_val = ''.join(c for c in val if c in '.-0123456789')
                    if clean_val:
                        gyro_data.append(float(clean_val))
                
                # Return in FEAGI format
                if len(gyro_data) >= 3:
                    return {
                        'gyro': {
                            '0': gyro_data[0],  # X
                            '1': gyro_data[1],  # Y
                            '2': gyro_data[2]   # Z
                        }
                    }
        
        except Exception as e:
            self.logger.warning(f"Error parsing sensor data: {e}")
        
        return {}
    
    def format_motors(self, feagi_output: dict) -> bytes:
        """
        Format FEAGI motor commands for Bittle.
        
        Bittle expects servo commands as: "i servo_id angle servo_id angle"
        """
        try:
            # Check for servo commands
            if 'servo' in feagi_output:
                command = "i"
                
                for feagi_id, angle in feagi_output['servo'].items():
                    # Map FEAGI ID to Bittle servo ID
                    bittle_id = self.SERVO_MAP.get(int(feagi_id))
                    if bittle_id is not None:
                        # Bittle expects angles offset by -90
                        bittle_angle = int(angle) - 90
                        command += f" {bittle_id} {bittle_angle}"
                
                if len(command) > 1:  # More than just "i"
                    return command.encode()
        
        except Exception as e:
            self.logger.warning(f"Error formatting motor command: {e}")
        
        return b''


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Bittle X Bluetooth Controller")
    parser.add_argument('--agent-id', default='bittle-001', help='Agent ID')
    parser.add_argument('--feagi-host', default='localhost', help='FEAGI host')
    parser.add_argument('--feagi-port', type=int, default=3000, help='FEAGI port')
    parser.add_argument('--platform', choices=['desktop', 'cloud'], help='Platform override')
    
    args = parser.parse_args()
    
    # Create and run robot
    robot = BittleRobot(
        agent_id=args.agent_id,
        feagi_host=args.feagi_host,
        feagi_port=args.feagi_port,
        platform=args.platform
    )
    
    # Run async
    try:
        asyncio.run(robot.run())
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")


if __name__ == "__main__":
    main()

