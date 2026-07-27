"""Read the error code the motors report, which the driver throws away.

Every Damiao feedback frame carries its state in the top nibble of byte 0:

    data[0] = (error << 4) | motor_id

`CanPacketDecoder::parse_motor_state_data` starts at data[1]. The nibble is
never read, so nothing in ros2_control, /joint_states, or /dynamic_joint_states
can report it — a motor sitting in a fault shows up only as a red LED and as
torque that never appears.

This listens. It opens a raw SocketCAN socket in read-only mode and decodes
frames the arm is already exchanging, so it is safe to run against a live
bringup: no frame is transmitted.
"""

import argparse
import socket
import struct
import time

# The bus runs CAN FD (the vendor CLI configures 1M/5M FD), and FD frames are
# invisible to a socket that has not opted in — a classic-CAN listener reads
# zero frames from a perfectly live arm. Opt in, and accept both frame sizes,
# since both start with the same can_id/len header.
CAN_FRAME = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME)
CANFD_FRAME = "=IBB2x64s"
CANFD_FRAME_SIZE = struct.calcsize(CANFD_FRAME)

#: Top nibble of byte 0, per the Damiao protocol. 1 is the healthy armed state;
#: the LED is green there and red for everything from 8 up.
STATE = {
    0x0: ("disabled", "토크 off — enable 안 됨"),
    0x1: ("enabled", "정상 (초록)"),
    0x8: ("over-voltage", "과전압"),
    0x9: ("under-voltage", "저전압 — 전원/전압 강하"),
    0xA: ("over-current", "과전류"),
    0xB: ("MOS over-temperature", "드라이버 과열"),
    0xC: ("rotor over-temperature", "모터 과열"),
    0xD: ("lost communication", "통신 두절"),
    0xE: ("overload", "과부하 — 한계에 눌려 있었음"),
}

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("interface", help="can0 is the right arm, can1 the left")
parser.add_argument("--seconds", type=float, default=3.0)
args = parser.parse_args()

sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
sock.bind((args.interface,))
sock.settimeout(0.5)

seen = {}
deadline = time.monotonic() + args.seconds
while time.monotonic() < deadline:
    try:
        frame = sock.recv(CANFD_FRAME_SIZE)
    except socket.timeout:
        continue
    if len(frame) == CANFD_FRAME_SIZE:
        can_id, length, _flags, data = struct.unpack(CANFD_FRAME, frame)
    elif len(frame) == CAN_FRAME_SIZE:
        can_id, length, data = struct.unpack(CAN_FRAME, frame)
    else:
        continue
    # Feedback frames only: the driver receives on send id + 0x10, so joints
    # 1..7 answer on 0x11..0x17 and the gripper (id 8) on 0x18. Commands go
    # the other way and carry no state.
    if not (0x11 <= (can_id & 0x1FFFFFFF) <= 0x18) or length < 1:
        continue
    joint = (can_id & 0x1FFFFFFF) - 0x10
    # Bytes 6 and 7 are the MOS and rotor temperatures in °C, already integral.
    # A latched over-temperature fault only means the motor WAS hot; whether it
    # is safe to clear depends on what the rotor reads now.
    seen.setdefault(joint, []).append((data[0] >> 4, data[6], data[7]))

if not seen:
    raise SystemExit(
        f"{args.interface}: 피드백 프레임이 없음. bringup이 떠 있는지, "
        "인터페이스가 맞는지 확인 (오른팔 can0, 왼팔 can1)"
    )

print(f"{args.interface} — {args.seconds:g}초 청취\n")
print(f"{'joint':<8}{'code':>6}  {'MOS°C':>7}{'Rtr°C':>7}  상태")
for joint in sorted(seen):
    codes = [code for code, _, _ in seen[joint]]
    worst = max(codes)  # a fault latches; report the worst seen, not the last
    _, t_mos, t_rotor = seen[joint][-1]
    name, korean = STATE.get(worst, (f"unknown 0x{worst:X}", "문서에 없는 코드"))
    flag = "  <<<" if worst not in (0x0, 0x1) else ""
    label = "gripper" if joint == 8 else f"j{joint}"
    print(f"{label:<8}{worst:>6X}  {t_mos:>7d}{t_rotor:>7d}  {name} — {korean}{flag}")

absent = [j for j in range(1, 8) if j not in seen]
if absent:
    print(
        "\n응답 없음: " + ", ".join(f"j{j}" for j in absent)
        + " — 이 모터는 피드백 자체를 안 보냄 (전원/CAN 배선/ID 확인)"
    )
