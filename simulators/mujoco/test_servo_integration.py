#!/usr/bin/env python3
"""Integration smoke test for SDK-only MuJoCo motor path."""

import sys
def test_with_sdk_only_surface():
    """Validate SDK exposes all primitives required by MuJoCo controller."""
    print("\n" + "=" * 70)
    print("Integration Test with FEAGI SDK")
    print("=" * 70)
    
    try:
        import feagi  # noqa: F401
        from feagi.pns import brain_output
        from feagi.pns.outputs import ServoMotor, RotaryMotor
        print("\n✓ Successfully imported FEAGI SDK")
    except ImportError as e:
        print(f"\n✗ Failed to import FEAGI SDK: {e}")
        return 1

    # Test 1: Validate output classes without touching runtime cache state.
    print("\n[Test 1] Validating SDK motor output classes...")
    try:
        _ = ServoMotor
        _ = RotaryMotor
        if hasattr(ServoMotor, "register") and hasattr(RotaryMotor, "register"):
            print("✓ ServoMotor.register and RotaryMotor.register are available")
        else:
            print("✗ Motor output register methods missing")
            return 1
    except Exception as e:
        print(f"✗ Failed to validate SDK motor outputs: {e}")
        return 1

    # Test 2: Verify SDK wrappers required by MuJoCo runtime path.
    print("\n[Test 2] Validating SDK wrapper availability...")
    required_methods = [
        "register_sensor_units",
        "register_motor_groups",
        "write_sensor_scalar",
        "flush_sensory_bytes",
    ]
    for method_name in required_methods:
        if hasattr(brain_output, method_name):
            print(f"✓ brain_output.{method_name} available")
        else:
            print(f"✗ Missing brain_output.{method_name}")
            return 1
    
    print("\n" + "=" * 70)
    print("All SDK integration checks passed!")
    print("=" * 70)
    print("\nNOTE: Runtime decode/transport behavior still requires a live FEAGI test.")
    
    return 0


def main():
    return test_with_sdk_only_surface()


if __name__ == "__main__":
    sys.exit(main())
