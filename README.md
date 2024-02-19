# TicTacSPOT
This project is about a game of Tic Tac Toe played by a user and Boston Dynamics' SPOT robot. 

## Overview

The SPOT robot uses object detection and fiducial detection to interact with the game. It identifies where the game piece is and determines the best spot to place it on the game board. 

## Object Detection

Object detection is used to identify the game pieces. This involves recognizing and locating specific objects within the visual field of the robot. 

## Fiducial Detection

Fiducial detection is used to identify and locate the game board. Fiducial markers are objects placed in the scene that appear in the image and can be used as a reference point or measure.

## Gameplay

The game starts with the user making the first move. The SPOT robot then calculates the best move using a Tic Tac Toe algorithm and places its piece on the board. The game continues until there's a winner or the board is full.

## Future Work

We plan to improve the object and fiducial detection algorithms to increase the accuracy and speed of the game. We also aim to add more interactive features to make the game more engaging.
