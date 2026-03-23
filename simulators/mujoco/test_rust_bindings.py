#!/usr/bin/env python3
"""SDK surface smoke test for MuJoCo controller integration."""

import sys


def test_sdk_surface_loads():
    """Test that FEAGI SDK exposes required controller-facing APIs."""
    print("\n" + "=" * 70)
    print("SDK Surface Test - MuJoCo Controller")
    print("=" * 70)
    
    try:
        import feagi  # noqa: F401
        from feagi.pns import brain_output
        print("\n✓ Successfully imported FEAGI SDK")
    except ImportError as e:
        print(f"\n✗ Failed to import: {e}")
        return 1

    print("\n[Test 1] Checking brain_output controller-facing APIs...")
    required_methods = [
        "register_sensor_units",
        "register_motor_groups",
        "write_sensor_scalar",
        "flush_sensory_bytes",
    ]
    for method_name in required_methods:
        if hasattr(brain_output, method_name):
            print(f"✓ brain_output.{method_name} is available")
        else:
            print(f"✗ Missing brain_output.{method_name}")
            return 1
    
    print("\n" + "=" * 70)
    print("All SDK surface tests passed!")
    print("=" * 70)
    print("\nThe MuJoCo controller can rely on FEAGI SDK APIs directly.")
    
    return 0


def main():
    return test_sdk_surface_loads()


if __name__ == "__main__":
    sys.exit(main())
