

import math
import copy

X = "X"
O = "O"
EMPTY = None

# Creates an empty board
def initial_state():
    return [[EMPTY, EMPTY, EMPTY],
             [EMPTY, EMPTY, EMPTY],
             [EMPTY, EMPTY, EMPTY]]
    

# Returns player who has the next turn on a board
def player(board):
    X_count = 0
    O_count = 0
    
    for i in range(3):
        for j in range(3):
            if board[i][j] == X:
                X_count += 1
            elif board[i][j] == O:
                O_count += 1    
    return X if X_count <= O_count else O

# Returns set of all possible actions (i, j) available on the board
def actions(board):
    
    return    

# Returns the board that results from making move (i,j)
def result(board, action):
    
    
    return 

# Returns the winner of the game, if there is one
def winner(board):
    
    return

# Returns True if game is over, False otherwise
def terminal(board):
    
    
    return False

# Returns 1 if X has won the game, -1 if O has won, 0 otherwise
def utility(board):
    
    
    return

# Returns the optimal action for the current player on the board
def minimax(board):
    
    
    return
