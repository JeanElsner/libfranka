"""A fake control unit that speaks research interface protocol version 5.

It behaves like a Franka Emika Robot on system 4.2.1 or later, as far as
connecting and streaming state go:

  - A Connect offering any other version is rejected with status
    kIncompatibleLibraryVersion and version 5, and the connection closed. That
    is what a real FER was observed to do.
  - A Connect offering version 5 is accepted, and the robot state is then
    streamed over UDP at 1 kHz to the port the client named.

Every field of the state carries a distinct value with a marker in its tenth
decimal, so a client that routes the state through the float-based current
protocol types, or reads a field at the wrong offset, gets caught. The joint
positions are those of a recorded FER configuration, which lets the client
check its model against Franka's for the same q.

    python fake_robot.py [--port 1337] [--version 5] [--seconds 30]
"""

import argparse
import socket
import struct
import threading
import time

# RobotState of protocol version 5: (field, struct code, count), packed, little
# endian. From panda-py's notes/state_layouts.py, which checks every offset
# against a C++ compiler reading libfranka-common e6aa0fc210d9.
LAYOUT_V5 = [
    ("message_id", "Q", 1), ("O_T_EE", "d", 16), ("O_T_EE_d", "d", 16), ("F_T_EE", "d", 16),
    ("EE_T_K", "d", 16), ("F_T_NE", "d", 16), ("NE_T_EE", "d", 16), ("m_ee", "d", 1),
    ("I_ee", "d", 9), ("F_x_Cee", "d", 3), ("m_load", "d", 1), ("I_load", "d", 9),
    ("F_x_Cload", "d", 3), ("elbow", "d", 2), ("elbow_d", "d", 2), ("tau_J", "d", 7),
    ("tau_J_d", "d", 7), ("dtau_J", "d", 7), ("q", "d", 7), ("q_d", "d", 7), ("dq", "d", 7),
    ("dq_d", "d", 7), ("ddq_d", "d", 7), ("joint_contact", "d", 7), ("cartesian_contact", "d", 6),
    ("joint_collision", "d", 7), ("cartesian_collision", "d", 6),
    ("tau_ext_hat_filtered", "d", 7), ("O_F_ext_hat_K", "d", 6), ("K_F_ext_hat_K", "d", 6),
    ("O_dP_EE_d", "d", 6), ("O_ddP_O", "d", 3), ("elbow_c", "d", 2), ("delbow_c", "d", 2),
    ("ddelbow_c", "d", 2), ("O_T_EE_c", "d", 16), ("O_dP_EE_c", "d", 6), ("O_ddP_EE_c", "d", 6),
    ("theta", "d", 7), ("dtheta", "d", 7), ("motion_generator_mode", "B", 1),
    ("controller_mode", "B", 1), ("errors", "?", 41), ("reflex_reason", "?", 41),
    ("robot_mode", "B", 1), ("control_command_success_rate", "d", 1),
]
STATE_FORMAT = "<" + "".join(f"{count}{code}" for _, code, count in LAYOUT_V5)
assert struct.calcsize(STATE_FORMAT) == 2373

HEADER = struct.Struct("<III")  # command, command id, size
CONNECT_REQUEST = struct.Struct("<HH")  # version, udp port
CONNECT_RESPONSE = struct.Struct("<BH")  # status, version
K_CONNECT, K_SUCCESS, K_INCOMPATIBLE = 0, 0, 1

# A recorded FER configuration, and Franka's gravity for it with no payload.
Q = [0.7936381933528991, -0.8116399619540988, -2.6598748181993264, -3.022184038143356,
     1.8152757280698988, 3.423588526337011, 0.6179116662605004]
IDENTITY = [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]


def marker(field_index, element):
    """A value unique to the field and element, with a 1e-10 precision marker."""
    return field_index + element / 100 + (field_index + 1) * 1e-10


def state_values(message_id):
    values = {}
    for index, (name, code, count) in enumerate(LAYOUT_V5):
        if code == "d":
            values[name] = [marker(index, e) for e in range(count)]
        elif code == "?":
            values[name] = [False] * count
    values.update(
        message_id=message_id,
        q=list(Q),
        # Frames the model uses must be valid transforms; no payload.
        F_T_EE=list(IDENTITY), EE_T_K=list(IDENTITY), F_T_NE=list(IDENTITY),
        NE_T_EE=list(IDENTITY),
        O_ddP_O=[0.0, 0.0, -9.81],  # the gravity vector, as robots report it
        m_ee=0.0, m_load=0.0, F_x_Cee=[0.0] * 3, F_x_Cload=[0.0] * 3, I_ee=[0.0] * 9,
        I_load=[0.0] * 9,
        motion_generator_mode=0,  # kIdle
        controller_mode=3,  # kOther
        robot_mode=1,  # kIdle
        control_command_success_rate=0.75 + 1e-10,
    )
    values["errors"][7] = True  # one error set, to check the error mapping
    flat = []
    for name, code, count in LAYOUT_V5:
        value = values[name]
        flat += value if count > 1 else [value]
    return struct.pack(STATE_FORMAT, *flat)


def stream(address, stop):
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    message_id = 1
    while not stop.is_set():
        udp.sendto(state_values(message_id), address)
        message_id += 1
        time.sleep(0.001)


def recv_exactly(conn, size):
    data = b""
    while len(data) < size:
        chunk = conn.recv(size - len(data))
        if not chunk:
            raise ConnectionError("client closed the connection")
        data += chunk
    return data


def serve(port, version, seconds):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", port))
    server.listen()
    server.settimeout(seconds)
    print("ready", flush=True)
    stop = threading.Event()
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            try:
                conn, peer = server.accept()
            except socket.timeout:
                break
            command, command_id, size = HEADER.unpack(recv_exactly(conn, HEADER.size))
            body = recv_exactly(conn, size - HEADER.size)
            offered, udp_port = CONNECT_REQUEST.unpack(body[: CONNECT_REQUEST.size])
            accepted = command == K_CONNECT and offered == version
            status = K_SUCCESS if accepted else K_INCOMPATIBLE
            conn.sendall(
                HEADER.pack(command, command_id, HEADER.size + CONNECT_RESPONSE.size)
                + CONNECT_RESPONSE.pack(status, version)
            )
            print(f"connect offering {offered}: {'accepted' if accepted else 'rejected'}",
                  flush=True)
            if not accepted:
                conn.close()
                continue
            threading.Thread(target=stream, args=((peer[0], udp_port), stop), daemon=True).start()
            # Hold the session open until the client leaves or time runs out.
            conn.settimeout(max(0.1, end - time.monotonic()))
            try:
                while conn.recv(4096):
                    pass
            except (socket.timeout, OSError):
                pass
            stop.set()
            conn.close()
            break
    finally:
        stop.set()
        server.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=1337)
    parser.add_argument("--version", type=int, default=5)
    parser.add_argument("--seconds", type=float, default=30)
    args = parser.parse_args()
    serve(args.port, args.version, args.seconds)
