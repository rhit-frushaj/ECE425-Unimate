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

def isValid(point, mat):
    return (0 <= point.x < 4) and (0 <= point.y < 4) and not (mat[point.x][point.y])

def BFS(mat, start, goal):
    r = len(mat)
    c = len(mat[0])
    
    # If Source and destination are valid
    if mat[start.x][start.y] or mat[goal.x][goal.y]: 
        return -1, []

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
            if isValid(Point(nx, ny), mat) and not visited[nx][ny]:
                visited[nx][ny] = True
                q.append(QNode(Point(nx, ny), d + 1, node))  # Pass current node as parent
                
    return -1, []

oGrid = [[0, 99, 99, 0], [0, 0, 0, 0], [0, 99, 99, 0], [0, 99, 0, 0]]

start = Point(0, 3)
goal = Point(3, 0)
distance, path = BFS(oGrid, start, goal)

oGrid[start.x][start.y] = 1
oGrid[goal.x][goal.y] = 2
print(oGrid[0])
print(oGrid[1])
print(oGrid[2])
print(oGrid[3])
print(f"Distance: {distance}")
print(f"Path: {path}")