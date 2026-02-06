# A point in a Maze (Needed for QNode)
class Point:
    def __init__(self, x_, y_):
        self.x = x_
        self.y = y_

# A QNode (Needed for BFS)
class QNode:
    def __init__(self, p_, d_):
        self.p = p_
        self.d = d_

def isValid(point,mat):
    return (0 <= point.x < 4) and (0 <= point.y < 4) and not (mat[point.x][point.y]) # map is 4 tiles, does not include 4

def BFS(mat, start, goal):
    r = len(mat)
    c = len(mat[0])
    
    # If Source and destination are valid
    if mat[start.x][start.y] or mat[goal.x][goal.y]: return -1

    # Do BFS using Queue and Visited
    visited = [[False] * c for _ in range(r)]
    from collections import deque
    q = deque([QNode(start, 0)])
    visited[start.x][start.y] = True
    while q:
        
        # Pop an item from queue
        node = q.popleft()
        p = node.p
        d = node.d

        # If we reached the goalination
        if p.x == goal.x and p.y == goal.y: return d
        
        # Try all four adjacent
        dx = [-1, 0, 0, 1]
        dy = [0, -1, 1, 0]
        for i in range(4):
            nx, ny = p.x + dx[i], p.y + dy[i]
            if isValid(Point(nx, ny),mat) and not visited[nx][ny]:
                visited[nx][ny] = True
                q.append(QNode(Point(nx, ny), d 
                               + 1))
                
    return -1
oGrid = [[0, 99, 99, 0], [0, 0, 0, 0], [0, 99, 99, 0], [0, 99, 0, 0]]

TMap1 = [[9, 5, 5, 3], [14, 15, 15, 10], [15, 9, 5, 6], [13, 6, 15, 15]]
TMap2 = [[11, 15, 15, 11], [8, 5, 5, 2], [10, 15, 15, 10], [14, 13, 5, 6]]

start = Point(0, 3)
goal = Point(3, 0)
a = BFS(oGrid, start, goal)

oGrid[start.x][start.y] = 1
oGrid[goal.x][goal.y] = 2
print(oGrid[0])
print(oGrid[1])
print(oGrid[2])
print(oGrid[3])
print(a)