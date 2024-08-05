import copy

class BoardInput:

    #initialize values
    def __init__(self, initial_values=None):
        self.totalXPieces = 0
        self.totalOPieces = 0
        self.totalCountPieces = 0

        if initial_values is None:
            # Didn't use list as default argument because mutable default arguments are shared across instances (They're not re-initalized for each instance)
            initial_values = [1,2,3,4,5,6,7,8,9]
            
        self.initialBoardState = [
            [initial_values[0], initial_values[1], initial_values[2]],
            [initial_values[3], initial_values[4], initial_values[5]],
            [initial_values[6], initial_values[7], initial_values[8]]
            ]
        self.boardState = copy.deepcopy(self.initialBoardState)
        self.previousBoardState = copy.deepcopy(self.boardState)         

    
    #Call this function to change the ids initially. Default is 0,1,2,3,4,5,6,7,8
    def changeInitialState(self, initial_values):
        self.boardState = [
            [initial_values[0], initial_values[1], initial_values[2]],
            [initial_values[3], initial_values[4], initial_values[5]],
            [initial_values[6], initial_values[7], initial_values[8]]
        ]
        self.previousBoard = copy.deepcopy(self.boardState)
        self.initialBoardState = copy.deepcopy(self.boardState)
    
    def printBoard(self):
        for row in self.boardState:
            for element in row:
                print(element, end=" ")
            print()

    def printPreviousBoard(self):
        for row in self.previousBoardState:
            for element in row:
                print(element, end=" ")
            print()

    def addXPiece(self):
        self.totalXPieces += 1
        self.totalCountPieces += 1

    def addOPiece(self):
        self.totalOPieces += 1
        self.totalCountPieces += 1

    def printBoardInfo(self):
        print("Previous Board State:")
        self.printPreviousBoard()
        print("Current Board State:")
        self.printBoard()
        print("\nTotal X Pieces: ", self.totalXPieces)
        print("Total O Pieces: ", self.totalOPieces)
        print("Total Count of Pieces: ", self.totalCountPieces)

    def updateBoard(self, currentIDs, move):
        newBoardState = copy.deepcopy(self.boardState)

        # Flatten the initial board state to get a list of IDs
        flat_initial_ids = sum(self.initialBoardState, [])

        # Find the ID that is no longer detected
        missingId = set(flat_initial_ids) - set(currentIDs)
        
        if missingId:
            missingId = missingId.pop()  # Get the missing ID

            for i in range(3):
                for j in range(3):
                    if self.initialBoardState[i][j] == missingId and self.boardState[i][j] not in ['X', 'O']:
                        newBoardState[i][j] = move
                        if(move == 'O'):
                            self.addOPiece()
                        elif move == 'X':
                            self.addXPiece()
                        else:
                            print(f"Move: '{move}' is neither 'X' or 'O")
                        # Update previous board
                        self.previousBoardState = copy.deepcopy(self.boardState) 
                        self.boardState = newBoardState
                        return

    #Getter functions
    def getXPieces(self):
        return self.totalXPieces

    def getOPieces(self):
        return self.totalOPieces

    def getTotalPieces(self):
        return self.totalCountPieces

    def getBoardState(self):
        return self.boardState
    
    def getPreviousBoardState(self):
        return self.previousBoardState