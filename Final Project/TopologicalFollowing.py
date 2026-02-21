# A point in a Maze (Needed for QNode)
class Point:
    def __init__(self, x_, y_):
        self.x = x_
        self.y = y_

# A QNode (Needed for BFS)
class QNode:
    def __init__(self, p_, d_, parent_=None):
        self.p = p_
        self.d = d_
        self.parent = parent_  # Track the parent node

def isValid(newpoint, currentpoint, mat):
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

def BFS(mat, start, goal):
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
            
            if isValid(Point(nx, ny), Point(ox, oy), mat) and not visited[nx][ny]:
                visited[nx][ny] = True
                q.append(QNode(Point(nx, ny), d + 1, node))  # Pass current node as parent
                                
    return -1, []

def solveMaze(grid = [[9, 7, 11, 15], [12, 1, 6, 11], [13, 0, 5, 2], [15, 12, 7, 14]], start = Point(0,0), goal = Point(3,3), toggle = 1):

    distance, path = BFS(grid, start, goal)

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
    return commandVector


print(f"CommandPath: {solveMaze()}")