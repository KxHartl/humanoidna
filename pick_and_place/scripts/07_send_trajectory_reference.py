import socket

HOST = "192.168.208.128" # IP adresa - definirati
PORT = 30003  # real-time port

FILE_PATH = "trajectory_task_space.txt"

def send_trajectory():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))

    print("Spojen na robota")

    with open(FILE_PATH, "r") as f:
        lines = f.readlines()

    # start URScript program
    s.send(b"def prog():\n")

    for line in lines:
        if line.strip() == "":
            continue
        if "t=" in line:
            line = line.split(", t=")[0] + ")\n"

        s.send(line.encode())

    s.send(b"end\n")

    s.close()

    print("Poslano")


if __name__ == "__main__":
    send_trajectory()