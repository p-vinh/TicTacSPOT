import math
import copy

X = "X"
O = "O"
# ids for the qr codes:
EMPTY = [0,1,2,3,4,5,6,7,8,9]

# Creates an empty board
def initial_state():
    return [[EMPTY, EMPTY, EMPTY],
             [EMPTY, EMPTY, EMPTY],
             [EMPTY, EMPTY, EMPTY]]
    

# Returns player who has the next turn on a board
def player(board):
    X_count = sum(x.count(X) for x in board)
    O_count = sum(x.count(O) for x in board)
      
    return O if X_count == O_count else X

# Returns set of all possible actions (i, j) available on the board
def actions(board):
    actions = set()  # Use a set instead of a list to store unique actions
    
    for i, row in enumerate(board):
        for j, cell in enumerate(row):
            if cell != "X" and cell != "O":
                actions.add((i, j))


    #Old code ---> Takes in a board with id numbers + X + O
    # for i, row in enumerate(board):
    #     for j, cell in enumerate(row):
    #         if cell in EMPTY:
    #             actions.add((i, j))

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
        if O_count == 3:
            return O
        elif X_count == 3:
            return X
    return None

# Returns True if game is over, False otherwise
def terminal(board):
    # If there is a winner, game is over
    if winner(board) is not None or not actions(board):
        return True
    return False

# Returns 1 if X has won the game, -1 if O has won, 0 otherwise
def utility(board):
    if winner(board) == X:
        return 1
    elif winner(board) == O:
        return -1
    else:
        return 0
    
# Returns the optimal action for the current player on the board
def minimax(board):
    optimal_action = None
    alpha = -math.inf
    beta = math.inf
    
    if terminal(board):
        return utility(board)
    
    if player(board) == X:
        bestScore = -math.inf
        
        for action in actions(board):
            score = minScore(result(board, action), alpha, beta)
            if bestScore < score:
                bestScore = score
                optimal_action = action
        return optimal_action, board[optimal_action[0]][optimal_action[1]]
    else:
        bestScore = math.inf
        
        for action in actions(board):
            score = maxScore(result(board, action), alpha, beta)
            
            if bestScore > score:
                bestScore = score
                optimal_action = action
        return optimal_action, board[optimal_action[0]][optimal_action[1]]
    
def maxScore(board, alpha, beta):
    if terminal(board):
        return utility(board)
    
    bestScore = -math.inf
    
    for action in actions(board):
        score = minScore(result(board, action), alpha, beta)
        bestScore = max(score, bestScore)
        
        alpha = max(alpha, bestScore)
        
        if beta <= alpha:
            break
    
    return bestScore

def minScore(board, alpha, beta):
    if terminal(board):
        return utility(board)
    
    bestScore = math.inf
    
    for action in actions(board):
        score = maxScore(result(board, action), alpha, beta)
        bestScore = min(score, bestScore)
        
        beta = min(beta, bestScore)
        
        if beta <= alpha:
            break

    return bestScore