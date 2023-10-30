
totalXPieces = 0
totalOPieces = 0
totalCountPieces = 0

# totalCountPieces + Empty Areas = 9
boardState = [[521, 522, 523],
              [524, 525, 546],
              [547, 548, 549]]

previousBoard = [[521, 522, 523],
                      [524, 525, 546],
                      [547, 548, 549]]

def updateTotalPieces(self):
    self.totalCountPieces = totalXPieces+totalOPieces

def checkValidInput(listOfId):
    if len(listOfId) + 1 + totalCountPieces == 9:
        return True
    else:
        return False
def spotUpdateBoard(move,self):
    print("SPOT Turn: Recording Move")
    newList = list(move)
    self.boardState[newList[0]][newList[1]] = 0
    print(self.boardState)
    print(self.previousBoard)
def updateBoard(listOfId,self):
    newBoardState = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]  # set empty new board state, 0 represents that area is taken

    if listOfId.__contains__(521):
        print("exists")
        newBoardState[0][0] = 521
    if listOfId.__contains__(522):
        newBoardState[0][1] = 522
    if listOfId.__contains__(523):
        newBoardState[0][2] = 523

    if listOfId.__contains__(524):
        newBoardState[1][0] = 524
    if listOfId.__contains__(525):
        newBoardState[1][1] = 525
    if listOfId.__contains__(546):
        newBoardState[1][2] = 546

    if listOfId.__contains__(547):
        newBoardState[2][0] = 547
    if listOfId.__contains__(548):
        newBoardState[2][1] = 548
    if listOfId.__contains__(549):
        newBoardState[2][2] = 549
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
    print("Changes")
    print(coordinatesThatChanged)
    return coordinatesThatChanged