

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
    X_count = sum(x.count(X) for x in board)
    O_count = sum(x.count(O) for x in board)
      
    return X if X_count <= O_count else O

# Returns set of all possible actions (i, j) available on the board
def actions(board):
    actions = set()
    
    for i, row in enumerate(board):
        for j, celll in enumerate(row):
            if cell == EMPTY:
                actions.add((i, j))
    return actions

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
