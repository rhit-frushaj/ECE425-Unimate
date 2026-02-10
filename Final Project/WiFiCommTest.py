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
# sock.connect((ARDUINO_IP, PORT))

def send(cmd):
    sock.sendall((cmd + "\n").encode())
    reply = sock.recv(1024).decode().strip()
    output.set(reply)

root = tk.Tk()
root.geometry("1000x700")
root.title("Arduino Control")

# Configure grid weights for responsiveness
num_rows = 5
num_cols = 9
row_weights = [1, 1, 1, 1, 15]
column_weights = [5, 5, 5, 5, 5, 1, 1, 1, 1]
for i in range(num_rows):
    for j in range(num_cols):
        root.grid_rowconfigure(i, weight=row_weights[i])
        root.grid_columnconfigure(j, weight=column_weights[j])



output = tk.StringVar()

# Place elements using grid (row, column)
label = tk.Label(root, textvariable=output)
label.grid(row=0, column=0, columnspan=2, pady=10)

# Buttons side by side at the top
tk.Button(root, text="Ping", command=lambda: send("PING")).grid(row=1, column=0, padx=10, pady=10)
# tk.Button(root, text="Status", command=lambda: send("STATUS")).grid(row=1, column=9, padx=10, pady=10)

# Movement Triad
triad_center = (2, 7)
tk.Button(root, text="↑", command=lambda: send("FORWARD")).grid(row=triad_center[0]-1, column=triad_center[1])
tk.Button(root, text="←", command=lambda: send("LEFT")).grid(row=triad_center[0], column=triad_center[1]-1)
tk.Button(root, text="→", command=lambda: send("RIGHT")).grid(row=triad_center[0], column=triad_center[1]+1)
tk.Button(root, text="↓", command=lambda: send("REVERSE")).grid(row=triad_center[0]+1, column=triad_center[1])

xlocation = tk.Label(root, text="x:").grid(row=1, column=9, padx=20)
xlocation = tk.Label(root, text="y:").grid(row=2, column=9, padx=20)


root.mainloop()
sock.close()


