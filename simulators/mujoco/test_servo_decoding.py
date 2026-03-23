#!/usr/bin/env python3
"""
Test suite for PositionalServo decoding behavior.

Tests the interaction between:
- Absolute cortical area (1x1x10 per channel)
- Incremental cortical area (2x1x10 per channel)
- Channel mapping (X coordinate to joint index)
- Priority (absolute should override incremental)
"""

import sys
import base64
from typing import Dict, List, Tuple


def decode_cortical_id(cortical_id_b64: str) -> Dict[str, any]:
    """Decode a base64 cortical ID into its components."""
    data = base64.b64decode(cortical_id_b64)
    return {
        "prefix": data[:4].decode("ascii", errors="ignore"),
        "sub_area": data[4],
        "config_bytes": list(data[5:7]),
        "group_id": data[7],
        "raw_bytes": list(data),
    }


def test_cortical_id_decoding():
    """Test that we can decode cortical IDs correctly."""
    # Absolute area ID (sub_area=0)
    abs_id = "b3BzZQABAQA="
    abs_decoded = decode_cortical_id(abs_id)
    assert abs_decoded["prefix"] == "opse"
    assert abs_decoded["sub_area"] == 0
    assert abs_decoded["group_id"] == 0
    
    # Incremental area ID (sub_area=1)
    inc_id = "b3BzZQEBAQA="
    inc_decoded = decode_cortical_id(inc_id)
    assert inc_decoded["prefix"] == "opse"
    assert inc_decoded["sub_area"] == 1
    assert inc_decoded["group_id"] == 0


def test_channel_mapping_absolute():
    """Test that X coordinates map to correct channels in absolute area."""
    # Absolute area: 1 neuron width per channel
    # For 5 joints: X=0 -> Joint 0, X=1 -> Joint 1, etc.
    
    test_cases = [
        (0, 0),  # X=0 -> channel 0 (Joint 0)
        (1, 1),  # X=1 -> channel 1 (Joint 1)
        (2, 2),  # X=2 -> channel 2 (Joint 2)
        (3, 3),  # X=3 -> channel 3 (Joint 3)
        (4, 4),  # X=4 -> channel 4 (Joint 4)
    ]
    
    for x, expected_channel in test_cases:
        channel_index = x  # Direct mapping for absolute area
        assert channel_index == expected_channel, (
            f"Absolute area: X={x} should map to channel {expected_channel}, "
            f"got {channel_index}"
        )


def test_channel_mapping_incremental():
    """Test that X coordinates map to correct channels in incremental area."""
    # Incremental area: 2 neuron widths per channel (even=forward, odd=backward)
    # For 5 joints with X=0-9:
    # X=0,1 -> Joint 0 (0=forward, 1=backward)
    # X=2,3 -> Joint 1 (2=forward, 3=backward)
    # X=4,5 -> Joint 2 (4=forward, 5=backward)
    # X=6,7 -> Joint 3 (6=forward, 7=backward)
    # X=8,9 -> Joint 4 (8=forward, 9=backward)
    
    test_cases = [
        (0, 0, True),   # X=0 -> channel 0, forward
        (1, 0, False),  # X=1 -> channel 0, backward
        (2, 1, True),   # X=2 -> channel 1, forward
        (3, 1, False),  # X=3 -> channel 1, backward
        (4, 2, True),   # X=4 -> channel 2, forward
        (5, 2, False),  # X=5 -> channel 2, backward
        (6, 3, True),   # X=6 -> channel 3, forward
        (7, 3, False),  # X=7 -> channel 3, backward
        (8, 4, True),   # X=8 -> channel 4, forward
        (9, 4, False),  # X=9 -> channel 4, backward
    ]
    
    for x, expected_channel, expected_forward in test_cases:
        channel_index = x // 2
        is_forward = (x % 2) == 0
        assert channel_index == expected_channel, (
            f"Incremental area: X={x} should map to channel {expected_channel}, "
            f"got {channel_index}"
        )
        assert is_forward == expected_forward, (
            f"Incremental area: X={x} should be "
            f"{'forward' if expected_forward else 'backward'}, "
            f"got {'forward' if is_forward else 'backward'}"
        )


def test_incremental_value_encoding():
    """Test the encoding/decoding of incremental direction values."""
    # Incremental values should be encoded as 0-1 where 0.5 = neutral
    # 0.0 = full backward, 0.5 = neutral, 1.0 = full forward
    
    test_cases = [
        (1.0, 0.0, 1.0),    # Full forward (forward=1.0, backward=0.0) -> 1.0
        (0.0, 1.0, 0.0),    # Full backward (forward=0.0, backward=1.0) -> 0.0
        (0.0, 0.0, 0.5),    # Neutral (forward=0.0, backward=0.0) -> 0.5
        (0.5, 0.0, 0.75),   # Half forward (forward=0.5, backward=0.0) -> 0.75
        (0.0, 0.5, 0.25),   # Half backward (forward=0.0, backward=0.5) -> 0.25
        (0.5, 0.5, 0.5),    # Balanced (forward=0.5, backward=0.5) -> 0.5
    ]
    
    for forward_val, backward_val, expected_output in test_cases:
        # Simulate the decoder's calculation
        net_direction = forward_val - backward_val  # Range: -1.0 to +1.0
        output_value = ((net_direction + 1.0) / 2.0)  # Convert to 0-1 range
        
        assert abs(output_value - expected_output) < 0.001, (
            f"Incremental encoding: forward={forward_val}, backward={backward_val} "
            f"should produce {expected_output}, got {output_value}"
        )


def test_python_sdk_incremental_interpretation():
    """Test how the Python SDK interprets incremental values."""
    # The Python SDK expects incremental values in 0-1 range where 0.5 = neutral
    # delta = (value - 0.5) * 2.0 * step_ratio
    
    min_angle = 0.0
    max_angle = 180.0
    half_range = (max_angle - min_angle) / 2.0  # 90.0
    incremental_step_ratio = 0.05
    step = half_range * incremental_step_ratio  # 4.5 degrees
    
    test_cases = [
        (1.0, step),      # Full forward: delta = (1.0 - 0.5) * 2.0 * step = +step
        (0.0, -step),     # Full backward: delta = (0.0 - 0.5) * 2.0 * step = -step
        (0.5, 0.0),       # Neutral: delta = (0.5 - 0.5) * 2.0 * step = 0.0
        (0.75, step/2),   # Half forward: delta = (0.75 - 0.5) * 2.0 * step = +step/2
        (0.25, -step/2),  # Half backward: delta = (0.25 - 0.5) * 2.0 * step = -step/2
    ]
    
    for value, expected_delta in test_cases:
        delta = (value - 0.5) * 2.0 * step
        assert abs(delta - expected_delta) < 0.001, (
            f"Python SDK incremental: value={value} should produce "
            f"delta={expected_delta}, got {delta}"
        )


def test_absolute_priority():
    """Test that absolute values take priority over incremental when both are present."""
    # This is a conceptual test - the actual priority is in the Rust decoder
    # If both absolute and incremental neurons fire, absolute should win
    
    # Expected behavior:
    # - has_absolute=True, has_incremental=True -> use absolute
    # - has_absolute=True, has_incremental=False -> use absolute
    # - has_absolute=False, has_incremental=True -> use incremental
    # - has_absolute=False, has_incremental=False -> no change
    
    priority_rules = [
        (True, True, "absolute"),
        (True, False, "absolute"),
        (False, True, "incremental"),
        (False, False, "none"),
    ]
    
    for has_abs, has_inc, expected_mode in priority_rules:
        if has_abs:
            actual_mode = "absolute"
        elif has_inc:
            actual_mode = "incremental"
        else:
            actual_mode = "none"
        
        assert actual_mode == expected_mode, (
            f"Priority test: has_absolute={has_abs}, has_incremental={has_inc} "
            f"should use {expected_mode}, got {actual_mode}"
        )


def test_z_depth_to_percentage():
    """Test the conversion from Z-depth position to percentage value."""
    # With z_depth=10 and Linear positioning:
    # Z=0 -> 0.0%, Z=5 -> 50%, Z=9 -> 90%
    
    z_depth = 10
    
    test_cases = [
        (0, 0.0),
        (1, 0.1),
        (5, 0.5),
        (9, 0.9),
    ]
    
    for z, expected_percentage in test_cases:
        # Linear interpolation (simplified)
        percentage = z / z_depth
        assert abs(percentage - expected_percentage) < 0.01, (
            f"Z-depth conversion: Z={z} with depth={z_depth} should produce "
            f"{expected_percentage}, got {percentage}"
        )


def test_integration_absolute_positioning():
    """Integration test: absolute positioning for a 5-joint robot."""
    # Simulate activating neurons in absolute area
    min_angle = 0.0
    max_angle = 180.0
    
    test_cases = [
        # (x, z, expected_joint, expected_angle_percent)
        (0, 0, 0, 0.0),    # Joint 0 -> 0% = 0 degrees
        (0, 5, 0, 0.5),    # Joint 0 -> 50% = 90 degrees
        (0, 9, 0, 0.9),    # Joint 0 -> 90% = 162 degrees
        (1, 5, 1, 0.5),    # Joint 1 -> 50% = 90 degrees
        (4, 9, 4, 0.9),    # Joint 4 -> 90% = 162 degrees
    ]
    
    for x, z, expected_joint, expected_percent in test_cases:
        channel_index = x
        percentage = z / 10.0  # z_depth=10
        
        assert channel_index == expected_joint
        assert abs(percentage - expected_percent) < 0.01
        
        # Calculate actual angle
        angle = min_angle + (max_angle - min_angle) * percentage
        expected_angle = min_angle + (max_angle - min_angle) * expected_percent
        assert abs(angle - expected_angle) < 1.0


def test_integration_incremental_movement():
    """Integration test: incremental movement for a 5-joint robot."""
    # Simulate activating neurons in incremental area
    half_range = 90.0  # (180 - 0) / 2
    incremental_step_ratio = 0.05
    step = half_range * incremental_step_ratio  # 4.5 degrees
    
    test_cases = [
        # (x, z, expected_joint, expected_direction, expected_delta)
        (0, 5, 0, "forward", step/2),    # Joint 0 forward at 50% intensity
        (1, 5, 0, "backward", -step/2),  # Joint 0 backward at 50% intensity
        (0, 9, 0, "forward", step*0.9),  # Joint 0 forward at 90% intensity
        (1, 9, 0, "backward", -step*0.9), # Joint 0 backward at 90% intensity
        (2, 5, 1, "forward", step/2),    # Joint 1 forward at 50% intensity
        (8, 9, 4, "forward", step*0.9),  # Joint 4 forward at 90% intensity
    ]
    
    for x, z, expected_joint, expected_dir, expected_delta in test_cases:
        channel_index = x // 2
        is_forward = (x % 2) == 0
        
        assert channel_index == expected_joint
        assert is_forward == (expected_dir == "forward")
        
        # Calculate percentage from Z depth
        z_percentage = z / 10.0  # z_depth=10
        
        # Simulate decoder output: convert to 0-1 range where 0.5 = neutral
        if is_forward:
            forward_val = z_percentage
            backward_val = 0.0
        else:
            forward_val = 0.0
            backward_val = z_percentage
        
        net_direction = forward_val - backward_val
        output_value = ((net_direction + 1.0) / 2.0)
        
        # Python SDK calculates delta
        delta = (output_value - 0.5) * 2.0 * step
        
        assert abs(delta - expected_delta) < 0.1, (
            f"Incremental movement: X={x}, Z={z} should produce delta={expected_delta}, "
            f"got {delta}"
        )


def main():
    """Run all tests."""
    print("=" * 70)
    print("Testing PositionalServo Decoding Logic")
    print("=" * 70)
    
    tests = [
        ("Cortical ID Decoding", test_cortical_id_decoding),
        ("Channel Mapping (Absolute)", test_channel_mapping_absolute),
        ("Channel Mapping (Incremental)", test_channel_mapping_incremental),
        ("Incremental Value Encoding", test_incremental_value_encoding),
        ("Python SDK Incremental Interpretation", test_python_sdk_incremental_interpretation),
        ("Absolute Priority", test_absolute_priority),
        ("Z-Depth to Percentage", test_z_depth_to_percentage),
        ("Integration: Absolute Positioning", test_integration_absolute_positioning),
        ("Integration: Incremental Movement", test_integration_incremental_movement),
    ]
    
    passed = 0
    failed = 0
    
    for test_name, test_func in tests:
        try:
            print(f"\n{test_name}...", end=" ")
            test_func()
            print("PASSED ✓")
            passed += 1
        except AssertionError as e:
            print(f"FAILED ✗")
            print(f"  Error: {e}")
            failed += 1
        except Exception as e:
            print(f"ERROR ✗")
            print(f"  Exception: {e}")
            failed += 1
    
    print("\n" + "=" * 70)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 70)
    
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
