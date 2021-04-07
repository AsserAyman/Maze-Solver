import queue
class Node:
    id = None  # Unique value for each node.
    up = None  # Represents value of neighbors (up, down, left, right).
    down = None
    left = None
    right = None
    previousNode = None  # Represents value of neighbors.
    edgeCost = None  # Represents the cost on the edge from any parent to this node.
    gOfN = None  # Represents the total edge cost
    hOfN = None  # Represents the heuristic value
    heuristicFn = None  # Represents the value of heuristic function

    def __init__(self, value):
        self.value = value


class SearchAlgorithms:

    path = []  # Represents the correct path from start node to the goal node.
    fullPath = []  # Represents all visited nodes from the start node to the goal node.
    totalCost = -1  # Represents the total cost in case using UCS, AStar (Euclidean or Manhattan)
    maze = [[]]
    heuristicMatrix = [[]]
    visited = [[]]
    prev = [[]]
    rowSize = 0
    colSize = 0
    def __init__(self, mazeStr, heristicValue=None, rowSize=5, colSize=7):
        self.rowSize = rowSize
        self.colSize = colSize
        self.visited = [[False] * colSize for _ in range(rowSize)]
        self.visitedStart = [[False] * colSize for _ in range(rowSize)]
        self.visitedEnd = [[False] * colSize for _ in range(rowSize)]
        self.prev = [[(-1,-1)] * colSize for _ in range(rowSize)]
        self.parentStart = [[(-1,-1)] * colSize for _ in range(rowSize)]
        self.parentEnd = [[(-1,-1)] * colSize for _ in range(rowSize)]
        self.prevDFS = [[(-1,-1)] * colSize for _ in range(rowSize)]
        self.dfsVisited = [[False] * colSize for _ in range(rowSize)]

        rows = mazeStr.split()
        for row in rows:
            self.maze.append(row.split(","))

        if heristicValue:
            for i in range(rowSize):
                temp = []
                for j in range(colSize):
                    x = i*colSize+j
                    temp.append(heristicValue[x])
                self.heuristicMatrix.append(temp)
            self.heuristicMatrix.pop(0)

    maze.pop(0)
    def getStartDim(self):
        for row in range(self.rowSize):
            for col in range(self.colSize):
                if self.maze[row][col] == 'S':
                    return (row, col)

    def getEndDim(self):
        for row in range(self.rowSize):
            for col in range(self.colSize):
                if self.maze[row][col] == 'E':
                    return (row, col)
    stack = []
    dfsVisited = [[]]
    prevDFS = [[]]
    def DLS(self,limit=50):
        self.path = []
        self.fullPath = []
        startRow, startColumn = self.getStartDim()
        self.exploreDFS(startRow, startColumn, limit)
        return self.path, self.fullPath

    def exploreDFS(self,row,col,limit):
        self.dfsVisited[row][col] = True
        self.fullPath.append((row, col))
        if limit == 0:
            return
        endR, endC = self.getEndDim()
        if row == endR and col == endC:
            self.backTrackDFS(row,col)
            return

        dirRow = [-1, 1, 0, 0]
        dirCol = [0, 0, -1, 1]
        for i in range(4):
            curRow = row + dirRow[i]
            curCol = col + dirCol[i]
            if curRow < 0 or curCol < 0:
                continue
            if curRow >= self.rowSize or curCol >= self.colSize:
                continue
            if self.dfsVisited[curRow][curCol]:
                continue
            if self.maze[curRow][curCol] == '#':
                continue
            self.prevDFS[curRow][curCol] = (row, col)
            self.exploreDFS(curRow,curCol,limit-1)

    def backTrackDFS(self,row,column):
        while not(self.maze[row][column] == 'S'):
            self.path.append((row,column))
            row, column = self.prevDFS[row][column]
        self.path.append((0, 0))
        self.path.reverse()


    visitedStart = [[]]
    visitedEnd = [[]]
    parentStart = [[]]
    parentEnd = [[]]
    qStart = []
    qEnd = []

    def BDS(self):
        self.path = []
        self.fullPath = []
        startDim = self.getStartDim()
        EndDim = self.getEndDim()
        x1, y1 = startDim
        x2, y2 = EndDim
        self.visitedStart[x1][y1] = True
        self.visitedEnd[x2][y2] = True
        self.qStart.append((x1,y1))
        self.qEnd.append((x2,y2))
        intersectionNode = (-1,-1)
        solved = False
        while self.qStart and self.qEnd:
            value1 = self.qStart.pop(0)
            a, b = value1
            self.fullPath.append((a, b))
            self.getPossibleNeigbors(a, b,True)

            value2 = self.qEnd.pop(0)
            c, d = value2
            self.fullPath.append((c, d))
            self.getPossibleNeigbors(c, d, False)

            if (self.visitedStart[a][b] == True and self.visitedEnd[a][b] == True):
                intersectionNode = (a,b)
                solved = True
                break

            if (self.visitedStart[c][d] == True and self.visitedEnd[c][d] == True):
                intersectionNode = (c,d)
                solved = True
                break
        if solved:
            self.getPath(intersectionNode,startNode=self.getStartDim(),endNode=self.getEndDim())
        return self.path, self.fullPath

    def getPath(self, intersectionNode,startNode,endNode):
        x,y = intersectionNode
        while (True):
            row,col = self.parentStart[x][y]
            self.path.insert(0,(row,col))
            x =row
            y = col
            if (row,col) == startNode:
                break
        self.path.append(intersectionNode)
        x, y = intersectionNode
        while (True):
            row, col = self.parentEnd[x][y]
            self.path.append((row,col))
            x = row
            y = col

            if (row, col) == endNode:
                break

    def getPossibleNeigbors(self, row, col, start):
        dirRow = [-1, 1, 0, 0]
        dirCol = [0, 0, -1, 1]
        for i in range(4):
            curRow = row + dirRow[i]
            curCol = col + dirCol[i]
            if curRow < 0 or curCol < 0:
                continue
            if curRow >= 5 or curCol >= 7:
                continue
            if self.visitedStart[curRow][curCol] and start == True:
                continue
            if self.visitedEnd[curRow][curCol] and start == False:
                continue
            if self.maze[curRow][curCol] == "#":
                continue

            if start==True:
                self.qStart.append((curRow, curCol))
                self.parentStart[curRow][curCol] = (row, col)
                self.visitedStart[curRow][curCol] = True

            else:
                self.qEnd.append((curRow, curCol))
                self.parentEnd[curRow][curCol] =(row, col)
                self.visitedEnd[curRow][curCol] = True



    open = queue.PriorityQueue()
    def BFS(self):
        self.path = []
        self.fullPath = []
        self.open.put((self.heuristicMatrix[0][0], (0, 0)))
        self.visited[0][0] = True

        while not(self.open.empty()):
            value, dim = self.open.get()
            x, y = dim
            self.fullPath.append((x, y))
            if self.maze[x][y] == 'E':
                self.backTrack(x,y)
                break
            self.exploreNeighbours(x,y)

        return self.path, self.fullPath, self.totalCost

    def backTrack(self,row,column):
        while not(self.maze[row][column] == 'S'):
            self.path.append((row,column))
            self.totalCost += self.heuristicMatrix[row][column]
            row, column = self.prev[row][column]
        self.path.append((0, 0))
        self.totalCost += 1
        self.path.reverse()

    def exploreNeighbours(self, row, col):
        dirRow = [-1, 1, 0, 0]
        dirCol = [0, 0, -1, 1]
        for i in range(4):
            curRow = row + dirRow[i]
            curCol = col + dirCol[i]
            if curRow < 0 or curCol < 0:
                continue
            if curRow >= self.rowSize or curCol >= self.colSize:
                continue
            if self.visited[curRow][curCol]:
                continue
            if self.maze[curRow][curCol] == '#':
                continue

            self.open.put((self.heuristicMatrix[curRow][curCol],(curRow,curCol)))
            self.visited[curRow][curCol] = True
            self.prev[curRow][curCol] = (row, col)

def main():
    searchAlgo = SearchAlgorithms('S,.,.,#,.,.,. .,#,.,.,.,#,. .,#,.,.,.,.,. .,.,#,#,.,.,. #,.,#,E,.,#,.')
    path, fullPath = searchAlgo.DLS()
    print('**DFS**\nPath is: ' + str(path) + '\nFull Path is: ' + str(fullPath) + '\n\n')

                #######################################################################################

    searchAlgo = SearchAlgorithms('S,.,.,#,.,.,. .,#,.,.,.,#,. .,#,.,.,.,.,. .,.,#,#,.,.,. #,.,#,E,.,#,.')
    path, fullPath = searchAlgo.BDS()
    print('**BFS**\nPath is: ' + str(path) + '\nFull Path is: ' + str(fullPath) + '\n\n')
                #######################################################################################

    searchAlgo = SearchAlgorithms('S,.,.,#,.,.,. .,#,.,.,.,#,. .,#,.,.,.,.,. .,.,#,#,.,.,. #,.,#,E,.,#,.', [0, 15, 2, 100, 60, 35, 30, 3
                                                                                                             , 100, 2, 15, 60, 100, 30, 2
                                                                                                             , 100, 2, 2, 2, 40, 30, 2, 2
                                                                                                             , 100, 100, 3, 15, 30, 100, 2
                                                                                                             , 100, 0, 2, 100, 30])
    path, fullPath, TotalCost = searchAlgo.BFS()
    print('** UCS **\nPath is: ' + str(path) + '\nFull Path is: ' + str(fullPath) + '\nTotal Cost: ' + str(
        TotalCost) + '\n\n')
               #######################################################################################




main()
