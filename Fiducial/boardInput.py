import copy

totalXPieces = 0
totalOPieces = 0
totalCountPieces = 0

# totalCountPieces + Empty Areas = 9
totalBoardState = [[0, 1, 2],
                    [3, 4, 5],
                    [6, 7, 8]]

boardState = [[0, 1, 2],
              [3, 4, 5],
              [6, 7, 8]]

xAndOBoard = [[0, 1, 2],
              [3, 4, 5],
              [6, 7, 8]]


def updateTotalPieces(self):
    self.totalCountPieces = totalXPieces+totalOPieces

def checkValidInput(listOfId):
    if len(listOfId) + totalCountPieces+1 == 9:
        return True
    else:
        return False

def spotUpdateBoard(move,self):
    print("SPOT Turn: Recording Move")
    newList = list(move)
    self.boardState[newList[0]][newList[1]] = 0
    print(self.boardState)
    print(self.previousBoard)

def updateBoard(listOfId,player,self):
    newBoardState = copy.deepcopy(boardState)  # set empty new board state, 0 represents that area is taken
    
    for i in range(3):
        for j in range(3):
            if not listOfId.__contains__(self.totalBoardState[i][j]) and self.boardState[i][j] != 'X' and self.boardState[i][j] != 'O':
                newBoardState[i][j] = player
    self.boardState = newBoardState

    

# Determines the difference between the previous board and the updated board
# Returns the coordinates of the array to update visual representation
def determineChanges(self):
    coordinatesThatChanged = []
    for i in range(3):
        for j in range(3):
            if self.previousBoard[i][j] != self.boardState[i][j]:
                coordinatesThatChanged.append((i, j))
    self.previousBoard = boardState
    return coordinatesThatChanged