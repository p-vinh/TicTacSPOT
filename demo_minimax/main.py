import pygame
import math
import tictactoe as ttt
import boardInput as bi

pygame.init()

# Screen
WIDTH = 300
ROWS = 3
win = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("TicTacToe")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Images
X_IMAGE = pygame.transform.scale(pygame.image.load("images/x.png"), (80, 80))
O_IMAGE = pygame.transform.scale(pygame.image.load("images/o.png"), (80, 80))

# Fonts
END_FONT = pygame.font.SysFont('arial', 40)


def draw_grid():
    gap = WIDTH // ROWS

    # Starting points
    x = 0
    y = 0

    for i in range(ROWS):
        x = i * gap

        pygame.draw.line(win, GRAY, (x, 0), (x, WIDTH), 3)
        pygame.draw.line(win, GRAY, (0, x), (WIDTH, x), 3)


def initialize_grid():
    dis_to_cen = WIDTH // ROWS // 2

    # Initializing the array
    game_array = [[None, None, None], [None, None, None], [None, None, None]]

    for i in range(len(game_array)):
        for j in range(len(game_array[i])):
            x = dis_to_cen * (2 * j + 1)
            y = dis_to_cen * (2 * i + 1)

            # Adding centre coordinates
            game_array[i][j] = (x, y, "", True)

    return game_array


def click(game_array):
    global x_turn, o_turn, images

    # Mouse position
    m_x, m_y = pygame.mouse.get_pos()


    for i in range(len(game_array)):
        for j in range(len(game_array[i])):
            x, y, char, can_play = game_array[i][j]

            # Distance between mouse and the centre of the square
            dis = math.sqrt((x - m_x) ** 2 + (y - m_y) ** 2)

            # If it's inside the square
            if dis < WIDTH // ROWS // 2 and can_play:
                if o_turn:  # If it's O's turn
                    images.append((x, y, O_IMAGE))
                    x_turn = True
                    o_turn = False
                    game_array[i][j] = (x, y, 'o', False)
                    print(game_array[i][j])




# Checking if someone has won
def has_won(game_array):
    # Checking rows
    for row in range(len(game_array)):
        if (game_array[row][0][2] == game_array[row][1][2] == game_array[row][2][2]) and game_array[row][0][2] != "":
            display_message(game_array[row][0][2].upper() + " has won!")
            return True

    # Checking columns
    for col in range(len(game_array)):
        if (game_array[0][col][2] == game_array[1][col][2] == game_array[2][col][2]) and game_array[0][col][2] != "":
            display_message(game_array[0][col][2].upper() + " has won!")
            return True

    # Checking main diagonal
    if (game_array[0][0][2] == game_array[1][1][2] == game_array[2][2][2]) and game_array[0][0][2] != "":
        display_message(game_array[0][0][2].upper() + " has won!")
        return True

    # Checking reverse diagonal
    if (game_array[0][2][2] == game_array[1][1][2] == game_array[2][0][2]) and game_array[0][2][2] != "":
        display_message(game_array[0][2][2].upper() + " has won!")
        return True

    return False


def has_drawn(game_array):
    for i in range(len(game_array)):
        for j in range(len(game_array[i])):
            if game_array[i][j][2] == "":
                return False

    display_message("It's a draw!")
    return True


def display_message(content):
    pygame.time.delay(500)
    win.fill(WHITE)
    end_text = END_FONT.render(content, 1, BLACK)
    win.blit(end_text, ((WIDTH - end_text.get_width()) // 2, (WIDTH - end_text.get_height()) // 2))
    pygame.display.update()
    pygame.time.delay(3000)


def render():
    win.fill(WHITE)
    draw_grid()

    # Drawing X's and O's
    for image in images:
        x, y, IMAGE = image
        win.blit(IMAGE, (x - IMAGE.get_width() // 2, y - IMAGE.get_height() // 2))

    pygame.display.update()


def main():
    global x_turn, o_turn, images, draw

    images = []
    draw = False

    run = True

    x_turn = False
    o_turn = True

    #--------------Initialize first----------------
    game_array = initialize_grid()
    print(game_array)
    
    LIST_IDS = [530, 531, 532, 533, 534, 535, 536, 537, 538]
    tempArr = [
            [LIST_IDS[0], LIST_IDS[1], LIST_IDS[2]],
            [LIST_IDS[3], LIST_IDS[4], LIST_IDS[5]],
            [LIST_IDS[6], LIST_IDS[7], LIST_IDS[8]]
        ]
    expectedNumberOfFiducials = 8
    player = ttt.O

    #Player expected to be first
    board = bi.BoardInput()
    board.changeInitialState(LIST_IDS)
    board.printBoard()
    #------------------------------------------------
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            if event.type == pygame.MOUSEBUTTONDOWN and o_turn:
                if has_won(game_array) or has_drawn(game_array):
                    run = False
                    print("Restarting game.....")
                    board = bi.BoardInput()
                    board.changeInitialState(LIST_IDS)
                else:
                    click(game_array)
                    #-------------update board--------------------
                    board.addOPiece()
                    
                    detectFiducial = []
                    for i in range(len(game_array)):
                        for j in range(len(game_array[i])):
                            x, y, char, can_play = game_array[i][j]
                            if can_play:
                                detectFiducial.append(tempArr[i][j])
                    
                    board.updateBoard(detectFiducial, player) #updates board

                    print("----------Player Turn-----------")
                    board.printBoard()
                    board.updateTotalPieces()
                    print("Number of X pieces:", board.getXPieces())
                    print("Number of O pieces:", board.getOPieces())
                    
                    player = ttt.X  
            
            if x_turn:  # If it's X's turn, aka SPOT's turn
                if has_won(game_array) or has_drawn(game_array):
                    run = False
                    print("Restarting game.....")
                    board = bi.BoardInput()
                    board.changeInitialState(LIST_IDS)
                else:
                    move, id = ttt.minimax(board.getBoardState())
                    board.spotUpdateBoard(move)
                    board.addXPiece()
                    x_turn = False
                    o_turn = True

                    i = move[0]
                    j = move[1]
                    x, y, char, can_play = game_array[i][j]
                    images.append((x, y, X_IMAGE))
                    game_array[i][j] = (x, y, 'x', False)

                    print("----------SPOT Turn-----------")
                    print("Spot's chosen move:", move)
                    board.printBoard()
                    board.updateTotalPieces()
                    print("Number of X pieces:", board.getXPieces())
                    print("Number of O pieces:", board.getOPieces())

                    player = ttt.O

        render()


while True:
    if __name__ == '__main__':
        main()
