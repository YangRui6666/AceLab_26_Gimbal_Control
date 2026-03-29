import argparse
import struct
import sys
import threading
import time
from dataclasses import dataclass
from queue import Empty, Queue

import serial

SOF = b"\xAA\x55"
EOF = b"\x5A\xA5"

CMD_GIMBAL_BOOT = 0x01
CMD_GIMBAL_HANDSHAKE_ACK = 0x02
CMD_GIMBAL_FEEDBACK = 0x03
CMD_GIMBAL_DISABLED = 0x08

CMD_VISION_HANDSHAKE_REQ = 0x81
CMD_VISION_SEARCH = 0x83
CMD_VISION_AUTO_AIM = 0x84
CMD_VISION_DISABLE = 0x87
CMD_VISION_UNLOCK = 0x88

MODE_DISABLED = 0
MODE_STANDBY = 1
MODE_SEARCH = 2
MODE_AUTO_AIM = 3
MODE_MANUAL = 4

SEARCH_PERIOD_S = 0.2
AUTO_AIM_PERIOD_S = 0.02


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    header = SOF + bytes([len(payload), cmd]) + payload
    return header + struct.pack("<H", crc16_modbus(header)) + EOF


@dataclass
class HostCommandState:
    mode: str = "idle"
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buffer.extend(data)
        frames: list[tuple[int, bytes]] = []

        while len(self.buffer) >= 8:
            if self.buffer[0:2] != SOF:
                del self.buffer[0]
                continue

            data_len = self.buffer[2]
            frame_len = 8 + data_len
            if len(self.buffer) < frame_len:
                break

            frame = bytes(self.buffer[:frame_len])
            if frame[-2:] != EOF:
                del self.buffer[0]
                continue

            expected_crc = struct.unpack("<H", frame[4 + data_len:6 + data_len])[0]
            actual_crc = crc16_modbus(frame[:4 + data_len])
            if expected_crc != actual_crc:
                del self.buffer[0]
                continue

            frames.append((frame[3], frame[4:4 + data_len]))
            del self.buffer[:frame_len]

        return frames


def mode_name(mode: int) -> str:
    mapping = {
        MODE_DISABLED: "DISABLED",
        MODE_STANDBY: "STANDBY",
        MODE_SEARCH: "SEARCH",
        MODE_AUTO_AIM: "AUTO_AIM",
        MODE_MANUAL: "MANUAL",
    }
    return mapping.get(mode, f"UNKNOWN({mode})")


def print_frame(cmd: int, payload: bytes) -> None:
    if cmd == CMD_GIMBAL_BOOT:
        print("RX 0x01 boot ready")
        return

    if cmd == CMD_GIMBAL_HANDSHAKE_ACK:
        if len(payload) != 4:
            print(f"RX 0x02 invalid length={len(payload)}")
            return
        timestamp_ms = struct.unpack("<I", payload)[0]
        print(f"RX 0x02 handshake ok timestamp={timestamp_ms}")
        return

    if cmd == CMD_GIMBAL_FEEDBACK:
        if len(payload) != 12:
            print(f"RX 0x03 invalid length={len(payload)}")
            return
        yaw, pitch, roll, timestamp_ms, mode, reserved = struct.unpack("<hhhIBB", payload)
        print(
            "RX 0x03 "
            f"yaw={yaw / 1000.0:.3f}deg "
            f"pitch={pitch / 1000.0:.3f}deg "
            f"roll={roll / 1000.0:.3f}deg "
            f"timestamp={timestamp_ms} "
            f"mode={mode_name(mode)} "
            f"reserved={reserved}"
        )
        return

    if cmd == CMD_GIMBAL_DISABLED:
        print("RX 0x08 disabled")
        return

    print(f"RX 0x{cmd:02X} payload={payload.hex()}")


def input_worker(queue: Queue[str]) -> None:
    while True:
        try:
            command = input().strip()
        except EOFError:
            queue.put("quit")
            return
        queue.put(command)
        if command == "quit":
            return


def send_search_keepalive(ser: serial.Serial) -> None:
    ser.write(build_frame(CMD_VISION_SEARCH))


def send_auto_aim_keepalive(ser: serial.Serial, yaw_deg: float, pitch_deg: float) -> None:
    payload = struct.pack(
        "<hhIBB",
        int(round(yaw_deg * 1000.0)),
        int(round(pitch_deg * 1000.0)),
        int(time.time() * 1000) & 0xFFFFFFFF,
        0,
        0,
    )
    ser.write(build_frame(CMD_VISION_AUTO_AIM, payload))


def handle_user_command(command: str, state: HostCommandState, ser: serial.Serial) -> bool:
    if command == "":
        return True

    if command == "quit":
        return False

    if command == "help":
        print("Commands: handshake, search, auto <yaw_deg> <pitch_deg>, disable, unlock, stop, quit")
        return True

    if command == "handshake":
        state.mode = "idle"
        ser.write(build_frame(CMD_VISION_HANDSHAKE_REQ))
        print("TX 0x81 handshake")
        return True

    if command == "search":
        state.mode = "search"
        send_search_keepalive(ser)
        print("TX 0x83 search keepalive enabled")
        return True

    if command.startswith("auto "):
        parts = command.split()
        if len(parts) != 3:
            print("Usage: auto <yaw_deg> <pitch_deg>")
            return True
        try:
            state.yaw_deg = float(parts[1])
            state.pitch_deg = float(parts[2])
        except ValueError:
            print("yaw_deg and pitch_deg must be numbers")
            return True
        state.mode = "auto"
        send_auto_aim_keepalive(ser, state.yaw_deg, state.pitch_deg)
        print(f"TX 0x84 auto keepalive enabled yaw={state.yaw_deg} pitch={state.pitch_deg}")
        return True

    if command == "disable":
        state.mode = "idle"
        ser.write(build_frame(CMD_VISION_DISABLE))
        print("TX 0x87 disable")
        return True

    if command == "unlock":
        state.mode = "idle"
        ser.write(build_frame(CMD_VISION_UNLOCK))
        print("TX 0x88 unlock")
        return True

    if command == "stop":
        state.mode = "idle"
        print("Periodic keepalive stopped")
        return True

    print("Unknown command. Type help for command list.")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="STM32 USB CDC protocol test host")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    parser.add_argument("--baudrate", type=int, default=115200, help="Nominal baudrate for pyserial")
    parser.add_argument("--timeout", type=float, default=0.05, help="Read timeout in seconds")
    args = parser.parse_args()

    state = HostCommandState()
    command_queue: Queue[str] = Queue()
    parser_state = FrameParser()

    try:
        ser = serial.Serial(args.port, args.baudrate, timeout=args.timeout)
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"Opened {args.port}. Type help for commands.")
    print("TX 0x81 handshake")
    ser.write(build_frame(CMD_VISION_HANDSHAKE_REQ))

    worker = threading.Thread(target=input_worker, args=(command_queue,), daemon=True)
    worker.start()

    next_search_at = time.monotonic()
    next_auto_aim_at = time.monotonic()

    try:
        while True:
            data = ser.read(256)
            if data:
                for cmd, payload in parser_state.feed(data):
                    print_frame(cmd, payload)

            now = time.monotonic()
            if state.mode == "search" and now >= next_search_at:
                send_search_keepalive(ser)
                next_search_at = now + SEARCH_PERIOD_S
            elif state.mode == "auto" and now >= next_auto_aim_at:
                send_auto_aim_keepalive(ser, state.yaw_deg, state.pitch_deg)
                next_auto_aim_at = now + AUTO_AIM_PERIOD_S

            try:
                while True:
                    command = command_queue.get_nowait()
                    if not handle_user_command(command, state, ser):
                        return 0
            except Empty:
                pass
    except KeyboardInterrupt:
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
