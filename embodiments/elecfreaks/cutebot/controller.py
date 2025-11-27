#!/usr/bin/env python
"""
Cutebot Bluetooth Controller

Simple controller for ELECFREAKS Cutebot using FEAGI Python SDK.
Demonstrates the simplicity of the Bluetooth robot framework!

Copyright 2016-2025 Neuraville Inc. All Rights Reserved.
"""

import asyncio
import argparse
from feagi.agent import BluetoothRobot


class CutebotRobot(BluetoothRobot):
    """
    ELECFREAKS Cutebot tri-wheeled robot controller.
    
    Only ~35 lines of robot-specific code!
    """
    
    # Bluetooth configuration (from embodiment.json)
    BLUETOOTH_CONFIG = {
        "device_name": "BBC micro:bit",
        "service_uuid": "6e400001-b5a3-f393-e0a9-e50e24dcca9e",
        "rx_uuid": "6e400003-b5a3-f393-e0a9-e50e24dcca9e",
        "tx_uuid": "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
    }
    
    def parse_sensors(self, raw_bytes: bytes) -> dict:
        """
        Parse Cutebot's sensor data.
        
        Cutebot (micro:bit) sends:
        - Gyro: "Gx,y,z#"
        - Accelerometer: "Ax,y,z#"
        - IR sensors: "I0,1#"
        - Ultrasonic: "Udist#"
        """
        try:
            data_str = raw_bytes.decode('utf-8').strip()
            
            if not '#' in data_str:
                return {}
            
            # Split and clean
            data_str = data_str.replace('\r', '').replace('\n', '')
            values_str = data_str.split('#')[0]
            
            # Parse based on prefix
            if values_str.startswith('G'):  # Gyro
                values = [float(v) for v in values_str[1:].split(',') if v]
                if len(values) >= 3:
                    return {
                        'gyro': {
                            '0': [values[0], values[1], values[2]]
                        }
                    }
            
            elif values_str.startswith('A'):  # Accelerometer
                values = [float(v) for v in values_str[1:].split(',') if v]
                if len(values) >= 3:
                    return {
                        'accelerometer': {
                            '0': [values[0], values[1], values[2]]
                        }
                    }
            
            elif values_str.startswith('I'):  # Infrared
                values = [int(v) for v in values_str[1:].split(',') if v]
                if len(values) >= 2:
                    return {
                        'infrared': {
                            '0': values[0],
                            '1': values[1]
                        }
                    }
            
            elif values_str.startswith('U'):  # Ultrasonic
                distance = float(values_str[1:])
                return {
                    'proximity': {
                        '0': distance
                    }
                }
        
        except Exception as e:
            self.logger.warning(f"Error parsing sensor data: {e}")
        
        return {}
    
    def format_motors(self, feagi_output: dict) -> bytes:
        """
        Format FEAGI motor commands for Cutebot.
        
        Cutebot expects motor commands as: "Mleft,right#"
        where left/right are -99 to 99
        """
        try:
            if 'motor' in feagi_output:
                left = int(feagi_output['motor'].get('0', 0))
                right = int(feagi_output['motor'].get('1', 0))
                
                # Clamp to -99 to 99
                left = max(-99, min(99, left))
                right = max(-99, min(99, right))
                
                command = f"M{left},{right}#"
                return command.encode()
        
        except Exception as e:
            self.logger.warning(f"Error formatting motor command: {e}")
        
        return b''


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Cutebot Bluetooth Controller")
    parser.add_argument('--agent-id', default='cutebot-001', help='Agent ID')
    parser.add_argument('--feagi-host', default='localhost', help='FEAGI host')
    parser.add_argument('--feagi-port', type=int, default=3000, help='FEAGI port')
    parser.add_argument('--platform', choices=['desktop', 'cloud'], help='Platform override')
    
    args = parser.parse_args()
    
    # Create and run robot
    robot = CutebotRobot(
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

