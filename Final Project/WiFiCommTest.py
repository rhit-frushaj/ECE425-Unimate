  
import socket
import tkinter as tk
import threading
from collections import deque

ARDUINO_IP = "172.20.10.4"
PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((ARDUINO_IP, PORT))
sock.setblocking(False)  # non-blocking socket

# -------------------------
# MAZE / COMMAND STATE
# -------------------------

# Holds the sequence of movement commands to send
commandQueue = deque()

# Event that fires when "OrderUp" is received from Arduino
order_up_event = threading.Event()

# Flag to stop the dispatch loop
dispatching = False

# -------------------------
# MAZE CLASSES & HELPERS
# -------------------------

class Point:
    """A point in a Maze (needed for QNode)."""
    def __init__(self, x_, y_):
        self.x = x_
        self.y = y_

class QNode:
    """
    Node structure for BFS. Holds a Point, the depth from the start,
    and a reference to the parent node for path reconstruction.
    """
    def __init__(self, p_, d_, parent_=None):
        self.p = p_
        self.d = d_
        self.parent = parent_

def isValidOccupancy(point, mat):
    """Returns True if the point is inside the grid and not a wall."""
    return (0 <= point.x < len(mat)) and (0 <= point.y < len(mat[0])) and not mat[point.x][point.y]

def BFSOccupancy(mat, start, goal):
    """
    Breadth-first search from start to goal on mat.
    Walls are any truthy cell value (e.g. 99 or True).
    Returns (distance, path) where path is a list of (x, y) tuples,
    or (-1, []) if no path exists.
    """
    r = len(mat)
    c = len(mat[0])

    if mat[start.x][start.y] or mat[goal.x][goal.y]:
        return -1, []

    visited = [[False] * c for _ in range(r)]
    q = deque([QNode(start, 0, None)])
    visited[start.x][start.y] = True

    while q:
        node = q.popleft()
        p = node.p

        if p.x == goal.x and p.y == goal.y:
            # Reconstruct path start → goal
            path = []
            current = node
            while current is not None:
                path.append((current.p.x, current.p.y))
                current = current.parent
            path.reverse()
            return node.d, path

        dx = [-1, 0, 0, 1]
        dy = [0, -1, 1, 0]
        for i in range(4):
            nx, ny = p.x + dx[i], p.y + dy[i]
            if isValidOccupancy(Point(nx, ny), mat) and not visited[nx][ny]:
                visited[nx][ny] = True
                q.append(QNode(Point(nx, ny), node.d + 1, node))

    return -1, []

def isValidTop(newpoint, currentpoint, mat):
    if newpoint.x < 0 or newpoint.y < 0 or newpoint.x > 3 or newpoint.y > 3:
        return 0
    if newpoint.x<currentpoint.x:
        a = ((mat[newpoint.x][newpoint.y]>>2 & 1) or (mat[currentpoint.x][currentpoint.y] & 1)) ^ 1 # If North of current point and South of new point are unoccupied
    elif newpoint.y>currentpoint.y:
        a = ((mat[newpoint.x][newpoint.y]>>3 & 1) or (mat[currentpoint.x][currentpoint.y]>>1 & 1)) ^ 1 # If East of current point and West of new point are unoccupied
    elif newpoint.x>currentpoint.x:
        a = ((mat[newpoint.x][newpoint.y] & 1) or (mat[currentpoint.x][currentpoint.y]>>2 & 1)) ^ 1 # If South of current point and North of new point are unoccupied
    elif newpoint.y<currentpoint.y:
        a = ((mat[newpoint.x][newpoint.y]>>1 & 1) or (mat[currentpoint.x][currentpoint.y]>>3 & 1)) ^ 1 # If West of current point and East of new point are unoccupied
    else:
        a = 0
        print("Error, direction input unusual")
    return (0 <= newpoint.x < 4) and (0 <= newpoint.y < 4) and a

def BFSTop(mat, start, goal):
    r = len(mat)
    c = len(mat[0])

    # Do BFS using Queue and Visited
    visited = [[False] * c for _ in range(r)]
    from collections import deque
    q = deque([QNode(start, 0, None)])
    visited[start.x][start.y] = True
    
    while q:
        # Pop an item from queue
        node = q.popleft()
        p = node.p
        d = node.d

        # If we reached the destination
        if p.x == goal.x and p.y == goal.y:
            # Reconstruct path
            path = []
            current = node
            while current is not None:
                path.append((current.p.x, current.p.y))
                current = current.parent
            path.reverse()  # Reverse to get start -> goal
            return d, path
        
        # Try all four adjacent
        dx = [-1, 0, 0, 1]
        dy = [0, -1, 1, 0]
        for i in range(4):
            nx, ny = p.x + dx[i], p.y + dy[i]
            if node.parent != None:
                ox = node.p.x
                oy = node.p.y
            else:
                ox = start.x
                oy = start.y
            
            if isValidTop(Point(nx, ny), Point(ox, oy), mat) and not visited[nx][ny]:
                visited[nx][ny] = True
                q.append(QNode(Point(nx, ny), d + 1, node))  # Pass current node as parent
                                
    return -1, []

def pathToCommands(path):
    """
    Convert a list of (x, y) coordinate pairs into Arduino movement commands.

    Grid orientation assumption:
      x increases → robot moves DOWN  (south)  → "b"
      x decreases → robot moves UP    (north)  → "FORWARD"
      y increases → robot moves RIGHT (east)   → "r"
      y decreases → robot moves LEFT  (west)   → "l"

    Adjust the mapping below if your robot's coordinate system differs.
    """
    DIR_MAP = {
        (-1,  0): "FORWARD",   # x decreases → forward / north
        ( 1,  0): "b",         # x increases → back    / south
        ( 0, -1): "l",         # y decreases → left    / west
        ( 0,  1): "r",         # y increases → right   / east
    }
    commands = []
    for i in range(1, len(path)):
        px, py = path[i - 1]
        cx, cy = path[i]
        delta = (cx - px, cy - py)
        cmd = DIR_MAP.get(delta)
        if cmd:
            commands.append(cmd)
        else:
            # Should never happen with a valid BFS path
            print(f"[WARN] Unexpected step delta {delta} at path index {i}")
    return commands

def solveMazeOccupancy(oGrid = [[0, 99, 99, 0], [0, 0, 0, 0], [0, 99, 99, 0], [0, 99, 0, 0]], start = Point(0,0), goal = Point(3,2), toggle = 1):
    global commandQueue, dispatching
    distance, path = BFSOccupancy(oGrid, start, goal)

    oGrid[start.x][start.y] = 1
    oGrid[goal.x][goal.y] = 2
    
    # toggle changes if the print data is printed (ie the map and the solution to the map) 1= on, 0 = off
    if(toggle):
        print(oGrid[0])
        print(oGrid[1])
        print(oGrid[2])
        print(oGrid[3])
        print(f"Distance: {distance}")
        print(f"Path: {path}")
    
    facing = 'd'
    commandVector = [0]*distance
    for i in range(distance):
        if path[i][0] < path[i+1][0]:
            if facing == 'u':
                facing = 'd'
                commandVector[i] = 'b'
            elif facing == 'd':
                commandVector[i] = 'f'
            elif facing == 'r':
                facing = 'd'
                commandVector[i] = 'r'
            elif facing == 'l':
                facing = 'd'
                commandVector[i] = 'l'
        if path[i][1] < path[i+1][1]:
            if facing == 'u':
                facing = 'r'
                commandVector[i] = 'r'
            elif facing == 'd':
                facing = 'r'
                commandVector[i] = 'l'
            elif facing == 'r':
                commandVector[i] = 'f'
            elif facing == 'l':
                facing = 'r'
                commandVector[i] = 'b'
        if path[i][0] > path[i+1][0]:
            if facing == 'u':
                commandVector[i] = 'f'
            elif facing == 'd':
                facing = 'u'
                commandVector[i] = 'b'
            elif facing == 'r':
                facing = 'u'
                commandVector[i] = 'l'
            elif facing == 'l':
                facing = 'u'
                commandVector[i] = 'r'
        if path[i][1] > path[i+1][1]:
            if facing == 'u':
                facing = 'l'
                commandVector[i] = 'l'
            elif facing == 'd':
                facing = 'l'
                commandVector[i] = 'r'
            elif facing == 'r':
                facing = 'l'
                commandVector[i] = 'b'
            elif facing == 'l':
                commandVector[i] = 'f'
                
    commandQueue = deque(commandVector)

    status_msg = f"Path found ({distance} steps). Starting dispatch…"
    output.set(status_msg)
    print(f"[INFO] {status_msg}")
    print(f"[INFO] Commands: {list(commandQueue)}")
    
    # Kick off the dispatch loop in a background thread so the GUI stays responsive
    dispatching = True
    threading.Thread(target=dispatch_loop, daemon=True).start()
    return True

def solveMazeTop(grid = [[9, 7, 11, 15], [12, 1, 6, 11], [13, 0, 5, 2], [15, 12, 7, 14]], start = Point(0,0), goal = Point(3,3), toggle = 1):
    global commandQueue, dispatching
    distance, path = BFSTop(grid, start, goal)

    grid[start.x][start.y] = 'S'
    grid[goal.x][goal.y] = 'T'
    
    # toggle changes if the print data is printed (ie the map and the solution to the map) 1= on, 0 = off
    if(toggle):
        print(grid[0])
        print(grid[1])
        print(grid[2])
        print(grid[3])
        print(f"Distance: {distance}")
        print(f"Path: {path}")
    
    facing = 'd'
    commandVector = [0]*distance
    for i in range(distance):
        if path[i][0] < path[i+1][0]:
            if facing == 'u':
                facing = 'd'
                commandVector[i] = 'b'
            elif facing == 'd':
                commandVector[i] = 'f'
            elif facing == 'r':
                facing = 'd'
                commandVector[i] = 'r'
            elif facing == 'l':
                facing = 'd'
                commandVector[i] = 'l'
        if path[i][1] < path[i+1][1]:
            if facing == 'u':
                facing = 'r'
                commandVector[i] = 'r'
            elif facing == 'd':
                facing = 'r'
                commandVector[i] = 'l'
            elif facing == 'r':
                commandVector[i] = 'f'
            elif facing == 'l':
                facing = 'r'
                commandVector[i] = 'b'
        if path[i][0] > path[i+1][0]:
            if facing == 'u':
                commandVector[i] = 'f'
            elif facing == 'd':
                facing = 'u'
                commandVector[i] = 'b'
            elif facing == 'r':
                facing = 'u'
                commandVector[i] = 'l'
            elif facing == 'l':
                facing = 'u'
                commandVector[i] = 'r'
        if path[i][1] > path[i+1][1]:
            if facing == 'u':
                facing = 'l'
                commandVector[i] = 'l'
            elif facing == 'd':
                facing = 'l'
                commandVector[i] = 'r'
            elif facing == 'r':
                facing = 'l'
                commandVector[i] = 'b'
            elif facing == 'l':
                commandVector[i] = 'f'
    commandQueue = deque(commandVector)

    status_msg = f"Path found ({distance} steps). Starting dispatch…"
    output.set(status_msg)
    print(f"[INFO] {status_msg}")
    print(f"[INFO] Commands: {list(commandQueue)}")
    
    # Kick off the dispatch loop in a background thread so the GUI stays responsive
    dispatching = True
    threading.Thread(target=dispatch_loop, daemon=True).start()
    return True
# -------------------------
# COMMAND DISPATCH LOOP
# -------------------------

def getNextCommand():
    """Returns the next command from the queue, or 'Stop' if the queue is empty."""
    if commandQueue:
        return commandQueue.popleft()
    return "Stop"

def dispatch_loop():
    """
    Sends one command at a time and waits for the Arduino to reply
    "OrderUp" before sending the next.  Stops after sending "Stop"
    (no reply expected for "Stop").
    """
    global dispatching

    while dispatching:
        cmd = getNextCommand()
        send(cmd)
        root.after(0, output.set, f"Sent: {cmd}")
        print(f"[SEND] {cmd}")

        if cmd == "Stop":
            root.after(0, output.set, "All done — Stop sent.")
            print("[INFO] Dispatch complete.")
            break

        # Block until the Arduino sends back "OrderUp"
        order_up_event.clear()
        print("[WAIT] Waiting for OrderUp…")
        order_up_event.wait()  # blocks this thread only; GUI remains live
        print("[INFO] OrderUp received — sending next command.")

    dispatching = False

# -------------------------
# RECEIVE THREAD
# -------------------------

def receive_data():
    """
    Continuously reads from the socket.  When "OrderUp" is detected,
    signals the dispatch loop so it can proceed to the next command.
    All other lines are shown in the GUI label.
    """
    buffer = ""
    while True:
        try:
            data = sock.recv(1024).decode()
            if data:
                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    print(f"[RECV] {line}")
                    if line == "OrderUp":
                        order_up_event.set()
                    else:
                        root.after(0, output.set, line)
        except BlockingIOError:
            pass
        except Exception as e:
            pass

threading.Thread(target=receive_data, daemon=True).start()

# -------------------------
# SEND FUNCTION
# -------------------------

def send(cmd):
    try:
        sock.sendall((cmd + "\n").encode())
    except Exception as e:
        print(f"[ERROR] Failed to send '{cmd}': {e}")

# -------------------------
# GUI LAYOUT
# -------------------------

root = tk.Tk()
root.geometry("1000x700")
root.title("Arduino Control")

output = tk.StringVar()
output.set("Waiting for LIDAR…")

num_rows = 5
num_cols = 9
row_weights    = [1, 1, 1, 1, 15]
column_weights = [5, 5, 5, 5, 5, 1, 1, 1, 1]

for i in range(num_rows):
    root.grid_rowconfigure(i, weight=row_weights[i])
for j in range(num_cols):
    root.grid_columnconfigure(j, weight=column_weights[j])

label = tk.Label(root, textvariable=output, font=("Arial", 24))
label.grid(row=0, column=0, columnspan=2, pady=10)

# ── Solve Maze buttons (replaces the old Ping button) ──────────────────────────
tk.Button(
    root,
    text="Solve Occupancy Maze",
    command=lambda: threading.Thread(target=solveMazeOccupancy, daemon=True).start()
).grid(row=1, column=0, padx=10, pady=10)

tk.Button(
    root,
    text="Solve Topological Maze",
    command=lambda: threading.Thread(target=solveMazeTop, daemon=True).start()
).grid(row=2, column=0, padx=10, pady=10)

# ── Directional triad ─────────────────────────────────────────────────────────
triad_center = (2, 7)
tk.Button(root, text="↑", command=lambda: send("FORWARD")).grid(row=triad_center[0]-1, column=triad_center[1])
tk.Button(root, text="←", command=lambda: send("l")).grid(row=triad_center[0],   column=triad_center[1]-1)
tk.Button(root, text="→", command=lambda: send("r")).grid(row=triad_center[0],   column=triad_center[1]+1)
tk.Button(root, text="↓", command=lambda: send("b")).grid(row=triad_center[0]+1, column=triad_center[1])

root.mainloop()
sock.close()
    