# Micro Servo Protocol Audit

## Purpose

This report records a source-level audit of `DHandControl/DH6Modbus/MicroServoControl.cpp` and `DHandControl/DH6Modbus/MicroServoControl.h` against `MicroServoControlProtocal.pdf`.

The goal is to separate PDF-compliant behavior from older or undocumented actuator commands before changing any low-level driver code. The driver files should be treated as frozen until hardware testing confirms which protocol variant the installed finger actuators actually expect.

## Short Summary

`MicroServoControl.cpp` appears to mix PDF-compliant register commands with older or undocumented commands.

The current PDF describes a register-based UART protocol:

- Request header: `0x55 0xAA`
- Response header: `0xAA 0x55`
- Read status command: `0x30`
- Read register command: `0x31`
- Write register command: `0x32`
- Register addresses are 2 bytes, little-endian
- Register data is little-endian
- Checksum is the low 8 bits of the sum of all bytes excluding the 2-byte header

The driver uses this shape correctly for `clearError()`, but other functions use command bytes such as `0x21`, `0x04`, `0x02`, and `0xF2`, plus one-byte control/register indexes. Those commands are not documented in the current PDF and may come from an older protocol or a vendor extension.

## Functions That Appear To Match The PDF

### `InitServo()`

`InitServo()` initializes the configured serial port at `DEFAULT_BAUDRATE`.

The PDF states that the factory default D-type UART baudrate is `921600`, with 8 data bits, 1 stop bit, and no parity. `DEFAULT_BAUDRATE 921600` should be kept for ESP32-to-finger-actuator communication. This baudrate is independent of the Python-to-ESP32 Modbus/RS485 baudrate.

### `calculateChecksum()`

The helper sums bytes starting at index `2`, which excludes the 2-byte request header. That matches the PDF checksum rule when callers pass the last byte index before the checksum.

The parameter name is slightly misleading because it is used as an inclusive end index, not a frame length.

### `clearError()`

`clearError()` appears PDF-compliant.

Expected PDF frame:

```text
55 AA 05 ID 32 18 00 01 00 Checksum
```

This writes value `0x0001` to register `0x0018`, the PDF's clear-fault command register. The implemented frame layout, byte order, data length, checksum range, and `Serial.write()` length are consistent with that format.

## Functions That Do Not Match The PDF

### `setPosition()`

Marking: PDF-inconsistent based on the current PDF.

Expected PDF-compliant target position write:

```text
55 AA 05 ID 32 29 00 Pos_L Pos_H Checksum
```

The implementation sends:

```text
55 AA 04 ID 21 37 Pos_L Pos_H Checksum
```

`0x21` is not a PDF command byte, and `0x37` is not the PDF target-position register. The current PDF uses write-register command `0x32` and target-position register `0x0029`.

### `setDeviceID()`

Marking: PDF-inconsistent based on the current PDF.

Expected PDF-compliant ID write:

```text
55 AA 05 ID 32 16 00 NewID 00 Checksum
```

The implementation sends:

```text
55 AA 03 ID 02 02 NewID Checksum
```

The PDF uses write-register command `0x32` and ID register `0x0016`. The implemented command/index pair is not documented in the current PDF.

### `ParameterSave()`

Marking: PDF-inconsistent based on the current PDF.

Expected PDF-compliant save command:

```text
55 AA 05 ID 32 1C 00 01 00 Checksum
```

The implementation sends:

```text
55 AA 03 ID 04 00 20 Checksum
```

The PDF uses write-register command `0x32` and save register `0x001C`. It also states that saving parameters returns two response frames, with the second frame indicating success.

### `getPosition()`

`getPosition()` is not implemented in the current driver.

A PDF-compliant current-position read would read register `0x002A`:

```text
55 AA 04 ID 31 2A 00 01 Checksum
```

Implementing this safely would also require response parsing, checksum validation, and timeout handling.

## Possible Older Protocol Or Vendor Extension

### `moveFingers()`

Marking: do not change until hardware-tested.

`moveFingers()` sends a broadcast frame with per-device payload entries:

```text
55 AA (3 * num + 1) FF F2 [ID Pos_L Pos_H]... Checksum
```

The PDF documents broadcast ID `0xFF`, but it does not document command `0xF2` or a packed multi-device payload format. This may be an older protocol command or an undocumented vendor extension for synchronized multi-finger control.

Because this function may be the only current synchronized multi-finger path, it should remain frozen until tested on hardware.

## Missing Or Weak Areas

- No serial response parsing is implemented.
- No timeout handling is implemented.
- No response checksum validation is implemented.
- Non-broadcast write commands ignore the status response frames described by the PDF.
- The header contains command constants that do not appear in the current PDF, including `0x01`, `0x02`, `0x03`, `0x04`, `0x21`, `0x22`, `0x23`, and `0xF2`.

## Proposed Hardware Validation Tests

### Single Finger `setPosition()` Test

Use the current `setPosition()` implementation on one known actuator ID.

Validate:

- The actuator moves to the expected target position.
- The position range matches the expected `0..2000` steps.
- No actuator error is raised.
- A logic analyzer or serial sniffer confirms the emitted frame.

### Multi-Finger `moveFingers()` Test

Use the current `moveFingers()` implementation with at least two actuator IDs and different target positions.

Validate:

- All addressed fingers move.
- Fingers start moving synchronously enough for hand gestures.
- The broadcast frame does not cause bus contention or unexpected responses.
- The command works repeatedly with at least 1 ms between command frames.

### `clearError()` Test

Trigger or simulate a clearable fault, then call `clearError()`.

Validate:

- The emitted frame matches the PDF write-register frame for register `0x18`.
- The actuator clears the fault.
- If possible, read status afterward to confirm the fault code is cleared.

### Optional PDF-Compliant Write-Register Set Position Test

Create a temporary test sketch or isolated test helper, not a production driver change, that sends:

```text
55 AA 05 ID 32 29 00 Pos_L Pos_H Checksum
```

Validate:

- The actuator accepts the PDF-compliant command.
- The actuator returns the expected response frame.
- Movement behavior matches or differs from the current `setPosition()` implementation.

This test determines whether the hardware expects the current PDF protocol, the older driver protocol, or both.

## Recommendation

Keep `MicroServoControl.cpp` and `MicroServoControl.h` frozen for now.

Continue improving the Modbus gateway layer and avoid changing low-level actuator driver behavior until hardware validation proves which serial protocol is correct for the installed finger actuators. In particular, do not alter `moveFingers()` until synchronized multi-finger behavior has been tested directly on hardware.
