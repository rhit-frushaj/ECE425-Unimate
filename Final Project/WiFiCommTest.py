# import socket

# ARDUINO_IP = "192.168.1.120"   # replace with Serial Monitor value
# PORT = 12345

# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# sock.connect((ARDUINO_IP, PORT))

# def send(cmd):
#     sock.sendall((cmd + "\n").encode())
#     reply = sock.recv(1024).decode().strip()
#     print("Arduino:", reply)

# send("PING")
# send("STATUS")
# send("HELLO")

# sock.close()

import socket
import tkinter as tk

ARDUINO_IP = "192.168.1.120"
PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((ARDUINO_IP, PORT))

def send(cmd):
    sock.sendall((cmd + "\n").encode())
    reply = sock.recv(1024).decode().strip()
    output.set(reply)

root = tk.Tk()
root.geometry("1000x1000")
root.title("Arduino Control")

output = tk.StringVar()
label = tk.Label(root, textvariable=output)
label.pack(pady=10)

tk.Button(root, text="Ping", command=lambda: send("PING")).pack()
tk.Button(root, text="Status", command=lambda: send("STATUS")).pack()

root.mainloop()
sock.close()


