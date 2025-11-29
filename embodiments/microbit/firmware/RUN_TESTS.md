# Running Tests

## Quick Start

```bash
cd embodiment-controllers/embodiments/microbit/firmware
cargo test
```

## Expected Output

```
running 15 tests
test bluetooth::tests::test_bluetooth_service_creation ... ok
test bluetooth::tests::test_process_received_data ... ok
test bluetooth::tests::test_parse_neuron_firing_packet_valid ... ok
test bluetooth::tests::test_parse_neuron_firing_packet_invalid_header ... ok
test bluetooth::tests::test_parse_neuron_firing_packet_incomplete ... ok
test bluetooth::tests::test_parse_neuron_firing_packet_max_coords ... ok
test bluetooth::tests::test_parse_neuron_firing_packet_too_many_coords ... ok
test bluetooth::tests::test_connection_status ... ok
test bluetooth::tests::test_get_capabilities_data ... ok
test bluetooth::tests::test_buffer_overflow_handling ... ok
test ble_stack::tests::test_create_advertising_data ... ok
test ble_stack::tests::test_create_advertising_data_with_name ... ok
test ble_stack::tests::test_create_advertising_data_long_name ... ok
test ble_stack::tests::test_create_scan_response ... ok
test ble_stack::tests::test_nus_uuid_format ... ok

test result: ok. 15 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out
```

## Test Details

### Run Specific Test Module
```bash
cargo test bluetooth::tests
cargo test ble_stack::tests
```

### Run with Output
```bash
cargo test -- --nocapture
```

### Run Single Test
```bash
cargo test test_parse_neuron_firing_packet_valid
```

## Troubleshooting

### If tests fail to compile:
1. Check Rust version: `rustc --version` (should be 1.70+)
2. Update dependencies: `cargo update`
3. Clean build: `cargo clean && cargo test`

### If tests pass but you see warnings:
- Warnings are OK, tests verify functionality
- Fix warnings separately if needed


