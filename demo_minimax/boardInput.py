import copy

class BoardInput:

    #initialize values
    def __init__(self):
        self.totalXPieces = 0
        self.totalOPieces = 0
        self.totalCountPieces = 0

        #Untouched inital board. Will only hold the ids of the board.
        self.totalBoardState = [[0, 1, 2],
                                [3, 4, 5],
                                [6, 7, 8]]

        # These values will be replaced by X or O, to keep track of the X and O pieces
        self.boardState = [[0, 1, 2],
                           [3, 4, 5],
                           [6, 7, 8]]

        self.previousBoard = copy.deepcopy(self.boardState)
    
    #Call this function to change the ids initially. Default is 0,1,2,3,4,5,6,7,8
    def changeInitialState(self, initial_values):
        self.boardState = [
            [initial_values[0], initial_values[1], initial_values[2]],
            [initial_values[3], initial_values[4], initial_values[5]],
            [initial_values[6], initial_values[7], initial_values[8]]
        ]
        self.previousBoard = copy.deepcopy(self.boardState)
        self.totalBoardState = copy.deepcopy(self.boardState)
    
    def printBoard(self):
        for row in self.boardState:
            for element in row:
                print(element, end=" ")
            print()
    
    def updateTotalPieces(self):
        self.totalCountPieces = self.totalXPieces + self.totalOPieces
    
    def addXPiece(self):
        self.totalXPieces += 1
    
    def addOPiece(self):
        self.totalOPieces += 1

    def checkValidInput(self, listOfId):
        if len(listOfId) + self.totalCountPieces + 1 == 9:
            return True
        else:
            return False

    def spotUpdateBoard(self, move):
        print("SPOT Turn: Recording Move")
        self.boardState[move[0]][move[1]] = 'X'
        print(self.boardState)
        print(self.previousBoard)

    def updateBoard(self, listOfId, player):
        newBoardState = copy.deepcopy(self.boardState)
    
        for i in range(3):
            for j in range(3):
                #If id is not detected, does not contain an X piece or O piece already, that is where the player most likely placed their piece
                #Update board
                if self.totalBoardState[i][j] not in listOfId and self.boardState[i][j] != 'X' and self.boardState[i][j] != 'O':
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
        self.previousBoard = self.boardState
        return coordinatesThatChanged


    #Getter functions
    def getXPieces(self):
        return self.totalXPieces

    def getOPieces(self):
        return self.totalOPieces

    def getTotalPieces(self):
        return self.totalCountPieces

    def getBoardState(self):
        return self.boardState
