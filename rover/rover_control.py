import socket
import time

ROVER_IP = "192.168.4.1"
ROVER_PORT = 4210

COMMANDS = {
    "w": "F",
    "s": "B",
    "a": "L",
    "d": "R",
    "q": "Q",
    "e": "E",
    "x": "S",
}

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

# Force packets through the rover network.
sock.bind(("192.168.4.2", 0))

print("RoverC-Pro Wireless Controller")
print("W: Forward | S: Backward")
print("A: Left    | D: Right")
print("Q: Rotate Left | E: Rotate Right")
print("X: Stop | quit: Exit")
print()

try:
    while True:
        key = input("Command: ").strip().lower()

        if key == "quit":
            break

        if key not in COMMANDS:
            print("Invalid command")
            continue

        command = COMMANDS[key].encode()

        # Send repeatedly during a short test.
        # Firmware watchdog stops after 400 ms.
        if key == "x":
            sock.sendto(
                command,
                (ROVER_IP, ROVER_PORT)
            )
            print("STOP")
        else:
            for _ in range(5):
                sock.sendto(
                    command,
                    (ROVER_IP, ROVER_PORT)
                )
                time.sleep(0.05)

            sock.sendto(
                b"S",
                (ROVER_IP, ROVER_PORT)
            )
            print("Command completed")

finally:
    for _ in range(3):
        sock.sendto(
            b"S",
            (ROVER_IP, ROVER_PORT)
        )
        time.sleep(0.02)

    sock.close()