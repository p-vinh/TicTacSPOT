

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
    # Check if action is valid
    if action not in actions(board):
        raise Exception("Invalid move")
    
    # Make a deep copy of the board, don't want to modify the original
    copy_board = copy.deepcopy(board)
    
    # Place the piece on the board
    copy_board[action[0]][action[1]] = player(board)
    return copy_board

# Returns the winner of the game, if there is one
def winner(board):
                     # Rows
    winning_combos = [[(0, 0), (0, 1), (0, 2)],
                      [(1, 0), (1, 1), (1, 2)],
                      [(2, 0), (2, 1), (2, 2)],
                      # Vertical
                      [(0, 0), (1, 0), (2, 0)],
                      [(0, 1), (1, 1), (2, 1)],
                      [(0, 2), (1, 2), (2, 2)],
                      # Diagonal
                      [(0, 0), (1, 1), (2, 2)],
                      [(0, 2), (1, 1), (2, 0)]]
    
    for wins in winning_combos:
        X_count = 0
        O_count = 0
        for i, j in wins:
            if board[i][j] == X:
                X_count += 1
            elif board[i][j] == O:
                O_count += 1
        if X_count == 3:
            return X
        elif O_count == 3:
            return O
    return None

# Returns True if game is over, False otherwise
def terminal(board):
    
    
    return False

# Returns 1 if X has won the game, -1 if O has won, 0 otherwise
def utility(board):
    
    
    return

# Returns the optimal action for the current player on the board
def minimax(board):
    
    
    return
