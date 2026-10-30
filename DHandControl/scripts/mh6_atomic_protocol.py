"""One atomic nine-channel packet, optionally followed by one bridge response.

No serial device is constructed here. Startup/clear-error operations continue
using Modbus; streaming packets use the same persistent UART under its lock.
"""
import struct
import time
import math

ADDRESS = 1
FUNCTION = 0x41
VERSION = 1
CAPABILITY_REGISTER = 50
CAPABILITIES = [0x4D36, VERSION, 7]
REQUEST_BYTES = 64
RESPONSE_BYTES = 12
ISSUED = 0x90
GUARDED_ACCEPTED = 0x95
MODES = ("legacy", "atomic_ack", "atomic_oneway")


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def with_crc(data):
    return data + struct.pack("<H", crc16(data))


def packet_words(packet):
    expected = {"finger_ids", "finger_positions", "palm_ids", "palm_positions", "palm_times"}
    if not isinstance(packet, dict) or set(packet) != expected:
        raise ValueError("atomic control requires a complete nine-channel packet")
    for prefix, count in (("finger", 5), ("palm", 4)):
        ids, positions = packet[prefix+"_ids"], packet[prefix+"_positions"]
        if (not isinstance(ids, (list, tuple)) or len(ids) != count
                or any(type(i) is not int for i in ids) or set(ids) != set(range(1, count+1))
                or not isinstance(positions, (list, tuple)) or len(positions) != count
                or any(type(p) is not int or not 0 <= p <= (2000 if prefix == "finger" else 1000)
                       for p in positions)):
            raise ValueError("invalid atomic "+prefix+" IDs/positions")
    times = packet["palm_times"]
    if not isinstance(times, (list, tuple)) or len(times) != 4 or any(type(t) is not int or not 1 <= t <= 65535 for t in times):
        raise ValueError("invalid atomic palm times")
    words = [0]*27
    words[0], words[11] = 5, 4
    for i, (device_id, position) in enumerate(zip(packet["finger_ids"], packet["finger_positions"])):
        words[1+2*i:3+2*i] = [device_id, position]
    for i, values in enumerate(zip(packet["palm_ids"], packet["palm_positions"], times)):
        words[12+3*i:15+3*i] = values
    return words


def encode_request(packet, sequence, *, acknowledge=True):
    if type(sequence) is not int or not 0 <= sequence <= 0xFFFFFFFF or type(acknowledge) is not bool:
        raise ValueError("invalid sequence or acknowledgement flag")
    return with_crc(struct.pack(">BBBBI27H", ADDRESS, FUNCTION, VERSION, int(acknowledge), sequence, *packet_words(packet)))


def decode_response(data, sequence):
    if len(data) != RESPONSE_BYTES or crc16(data[:-2]) != int.from_bytes(data[-2:], "little"):
        raise ValueError("atomic response length/CRC mismatch")
    address, function, version, flags, received, status = struct.unpack(">BBBBIH", data[:-2])
    if (address, function, version, flags) != (ADDRESS, FUNCTION, VERSION, 1) or received != sequence:
        raise ValueError("atomic response protocol/sequence mismatch")
    return status


class AtomicHandTransport:
    """Never retries motion; one-way success means local write completion only."""
    def __init__(self, hand, mode, timeout=.15):
        if mode not in MODES[1:] or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("invalid atomic transport mode/timeout")
        self.hand, self.mode, self.timeout = hand, mode, timeout
        self.sequence = time.monotonic_ns() & 0xFFFFFFFF
        self.verified = False

    def verify_firmware(self):
        # No motion: old firmware returns zeros at these reserved registers.
        self.verified = False
        with self.hand.transaction_lock:
            response = self.hand.client.read_holding_registers(address=CAPABILITY_REGISTER, count=3, device_id=1)
            self.hand._ensure_ok(response, "核对原子控制固件能力")
            if list(getattr(response, "registers", [])) != CAPABILITIES:
                raise RuntimeError("atomic control firmware mismatch; flash the matching DH6Modbus sketch before testing")
            state = self.hand.client.read_holding_registers(address=80, count=4, device_id=1)
            self.hand._ensure_ok(state, "核对 ESP32 手指限力保护配置")
            if list(getattr(state, "registers", [])) != [0x4647, 1, 0x1F03, 0x0204]:
                raise RuntimeError("finger guard is locked: fill and arm FingerGuardConfig.h before hardware motion")
            self.verified = True

    def send(self, packet):
        hand = self.hand
        result = {"status": "NOT_SENT", "stage": "validation", "execution_uncertain": False,
                  "transport_mode": self.mode, "sequence": self.sequence, "device_confirmed": False,
                  "host_write_completed": False,
                  "timings_ms": {}}
        hand.last_move_result = result
        try:
            # Enforce the same authoritative mechanical limits as legacy sends.
            hand._targets("F", packet["finger_ids"], packet["finger_positions"])
            hand._targets("M", packet["palm_ids"], packet["palm_positions"])
            data = encode_request(packet, self.sequence, acknowledge=self.mode == "atomic_ack")
        except (KeyError, TypeError, ValueError) as exc:
            result.update(status="REJECTED", error=str(exc))
            return False
        if not hand._motion_ready():
            return False
        if not self.verified:
            result.update(status="NOT_VERIFIED", error="firmware capability check is required")
            return False
        with hand.transaction_lock:
            if not hand.persistent_connection or not hand._client_connected():
                result.update(status="NOT_CONNECTED")
                return False
            serial = getattr(hand.client, "socket", None)
            if serial is None:
                result.update(status="NOT_CONNECTED", error="persistent serial socket is unavailable")
                return False
            old_timeout, old_write_timeout = serial.timeout, serial.write_timeout
            self.sequence = (self.sequence+1) & 0xFFFFFFFF
            try:
                serial.write_timeout = self.timeout
                result.update(stage="wire_write", execution_uncertain=True)
                started = time.perf_counter()
                try:
                    if serial.write(data) != len(data):
                        raise RuntimeError("partial atomic write; command will not be retried")
                    serial.flush()  # Drain the local TX queue; does not confirm reception.
                    result["host_write_completed"] = True
                finally:
                    result["timings_ms"]["wire_write"] = 1000*(time.perf_counter()-started)
                if self.mode == "atomic_oneway":
                    result.update(status="HOST_WRITTEN_UNCONFIRMED", stage="host_write_complete")
                    return True
                result["stage"] = "ack_wait"
                started = time.perf_counter()
                deadline = time.monotonic()+self.timeout
                received = bytearray()
                try:
                    while len(received) < RESPONSE_BYTES:
                        remaining = deadline-time.monotonic()
                        if remaining <= 0:
                            raise TimeoutError("atomic response timed out; command will not be retried")
                        serial.timeout = remaining
                        chunk = serial.read(RESPONSE_BYTES-len(received))
                        if not chunk:
                            raise TimeoutError("atomic response missing/incomplete; command will not be retried")
                        received.extend(chunk)
                    status = decode_response(bytes(received), result["sequence"])
                    result["firmware_status"] = status
                    if status != GUARDED_ACCEPTED:
                        # A correlated bridge rejection is known, not a missing ACK.
                        result["execution_uncertain"] = False
                        raise RuntimeError(f"atomic bridge rejected packet: 0x{status:04X}")
                    result.update(status="GUARDED_TARGET_ACCEPTED", execution_uncertain=False, device_confirmed=True)
                    return True
                finally:
                    result["timings_ms"]["ack_wait"] = 1000*(time.perf_counter()-started)
            except (OSError, RuntimeError, ValueError) as exc:
                result.update(status="FAILED", error=f"{type(exc).__name__}: {exc}")
                return False
            finally:
                serial.timeout, serial.write_timeout = old_timeout, old_write_timeout
