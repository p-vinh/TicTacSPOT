import boardInput as bi
import tictactoe as ttt

def main():

    #Ignore this file, this was me testing boardInput and tictactoe

    initial_values = [567,568,569,570,571,572,573,574,575]

    test = bi.BoardInput()
    test.changeInitialState(initial_values)
    test.printBoard()

    changed_values1 = [567,568,569,570,571,574]
    changed_values = [569,570,571,572,574] 

    print()
    print("Player made their move....")
    test.updateBoard(changed_values1,"O")
    test.updateBoard(changed_values, "X")
    test.printBoard()

    #How to ask for SPOT's minimax
    move = ttt.minimax(test.getBoardState())
    print(move)


if __name__ == '__main__':
    main()