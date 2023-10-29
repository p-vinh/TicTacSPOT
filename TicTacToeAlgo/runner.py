import pygame
import time
import sys

import tictactoe as ttt
import boardInput as inputId

# Initialize pygame
pygame.init()
size = width, height = 700, 700

# Colors
black = (0, 0, 0)
grey = (105, 105, 105)
white = (255, 255, 255)

# Create screen
screen = pygame.display.set_mode(size)

smallFont = pygame.font.Font(".\\OpenSans-Regular.ttf", 10)
mediumFont = pygame.font.Font(".\\OpenSans-Regular.ttf", 28)
largeFont = pygame.font.Font(".\\OpenSans-Regular.ttf", 40)
moveFont = pygame.font.Font(".\\OpenSans-Regular.ttf", 60)

user = None
board = ttt.initial_state()
ai_turn = False

listOfId = [[521, 522, 523], [524, 525, 546], [547, 548, 549]]  # board ids

# delete these later
user_text = 'TESTTT'
input_rect = pygame.Rect(width / 3 - 50, height - 100, 140, 32)
color = white

while True:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        if event.type == pygame.KEYDOWN:  # you can delete this later, this is so I can "input" the id codes
            if event.key == pygame.K_BACKSPACE:
                user_text = user_text[0:-1]
            else:
                user_text += event.unicode

    screen.fill(black)

    user = ttt.O

    # Choose player
    if user is None:

        # Title
        title = largeFont.render("Play Tic-Tac-Toe", True, white)
        titleRect = title.get_rect()
        titleRect.center = ((width / 2), 50)
        screen.blit(title, titleRect)

        # Buttons
        playXButton = pygame.Rect((width / 8), (height / 2), (width / 4), 50)
        playX = mediumFont.render("Play as X", True, black)
        playXRect = playX.get_rect()
        playXRect.center = playXButton.center
        pygame.draw.rect(screen, white, playXButton)
        screen.blit(playX, playXRect)

        playOButton = pygame.Rect(5 * (width / 8), (height / 2), (width / 4), 50)
        playO = mediumFont.render("Play as O", True, black)
        playORect = playO.get_rect()
        playORect.center = playOButton.center
        pygame.draw.rect(screen, white, playOButton)
        screen.blit(playO, playORect)

        # If button clicked, start game
        click, _, _ = pygame.mouse.get_pressed()
        if click == 1:
            mouse = pygame.mouse.get_pos()
            if playXButton.collidepoint(mouse):
                time.sleep(0.2)
                user = ttt.X
            elif playOButton.collidepoint(mouse):
                time.sleep(0.2)
                user = ttt.O

    # This whole if statement is skipped because I already initialized user = ttt.0
    else:
        # Draw game board
        tile_size = 100
        tile_origin = (width / 2 - (1.5 * tile_size),
                       height / 2 - (1.5 * tile_size))
        tiles = []

        for i in range(3):
            row = []
            for j in range(3):
                rect = pygame.Rect(tile_origin[0] + j * tile_size,
                                   tile_origin[1] + i * tile_size,
                                   tile_size, tile_size)
                pygame.draw.rect(screen, white, rect, 3)

                # Adds Id in center of rectangle
                id = largeFont.render(str(listOfId[i][j]), True, grey)
                idRect = id.get_rect()
                idRect.center = rect.center
                screen.blit(id, idRect)

                if board[i][j] != ttt.EMPTY:
                    move = moveFont.render(board[i][j], True, white)
                    moveRect = move.get_rect()
                    moveRect.center = rect.center
                    screen.blit(move, moveRect)
                row.append(rect)
            tiles.append(row)

        game_over = ttt.terminal(board)
        player = ttt.player(board)

        # Show title
        if game_over:
            winner = ttt.winner(board)

            if winner is None:
                title = f"Game Over: Tie."
            else:
                title = f"Game Over: {winner} wins."
        elif user == player:
            title = f"Play as {user}"
        else:
            title = f"Computer thinking..."

        title = largeFont.render(title, True, white)
        titleRect = title.get_rect()
        titleRect.center = ((width / 2), 30)
        screen.blit(title, titleRect)

        pygame.draw.rect(screen, white, input_rect, 2)
        # Remove this later, this is so I can "input" the ids
        text_surface = smallFont.render(user_text, True, white)
        screen.blit(text_surface, (input_rect.x + 5, input_rect.y + 5))
        input_rect.w = max(300, text_surface.get_width() + 10)

        # Button
        playXButton = pygame.Rect((width / 3), height - 50, (width / 4), 40)
        playX = mediumFont.render("Enter", True, black)
        playXRect = playX.get_rect()
        playXRect.center = playXButton.center
        pygame.draw.rect(screen, white, playXButton)
        screen.blit(playX, playXRect)

        # Check AI move
        if user != player and not game_over:
            if ai_turn:
                time.sleep(0.5)
                move = ttt.minimax(board)
                board = ttt.result(board, move)
                ai_turn = False

                #update board state
                inputId.totalXPieces = inputId.totalXPieces + 1  # updates amount of pieces
                inputId.updateTotalPieces(inputId)
                inputId.spotUpdateBoard(move,inputId)
                inputId.determineChanges(inputId) #determines changes and saves this move as previous board
                user_text = ""

            else:
                ai_turn = True

        # Check for a user move
        # click, _, _ = pygame.mouse.get_pressed()
        if user == player and not game_over:  # removed click ==1 as another condition

            # This is where it will detect fiducials and input it here
            # For the time being, I set it where the user has to input the id that are avaliable in the terminal

            click, _, _ = pygame.mouse.get_pressed()
            mouse = pygame.mouse.get_pos()
            if click == 1 and playXButton.collidepoint(mouse):
                print("Enter")

                # Change here to enter inputs
                idLook = user_text
                temp = idLook.split()
                idInput = []
                for i in temp:
                    idInput.append(int(i))
                print(idInput)

                # Waits till it detects amountOfPieces + ids that it can detect = 9
                if inputId.checkValidInput(idInput):
                    inputId.totalOPieces = inputId.totalOPieces + 1
                    inputId.updateTotalPieces(inputId)
                    inputId.updateBoard(idInput, inputId)
                    changes = inputId.determineChanges(inputId)
                    board = ttt.result(board, changes[0]) #there should only be one change
                else:
                    print("hmm something is not right..still updating the board..I'll wait")

            ''' Commented this out
            mouse = pygame.mouse.get_pos()
            for i in range(3):
                for j in range(3):
                    if (board[i][j] == ttt.EMPTY and tiles[i][j].collidepoint(mouse)):
                        board = ttt.result(board, (i, j))
            '''

        # Game Over Menu
        if game_over:
            againButton = pygame.Rect(width / 3, height - 65, width / 3, 50)
            again = mediumFont.render("Play Again", True, black)
            againRect = again.get_rect()
            againRect.center = againButton.center
            pygame.draw.rect(screen, white, againButton)
            screen.blit(again, againRect)
            click, _, _ = pygame.mouse.get_pressed()
            if click == 1:
                mouse = pygame.mouse.get_pos()
                if againButton.collidepoint(mouse):
                    time.sleep(0.2)
                    user = None
                    board = ttt.initial_state()
                    ai_turn = False

    pygame.display.flip()
