#MAIN GAMEPALY LOOP
import cv2 as cv
import numpy as np
import argparse
from queue import Queue
import tictactoe as ttt
import boardInput as bi
import logging
import math
import signal
import sys
import threading
from dotenv import load_dotenv
import time
from sys import platform
from PIL import Image
import bosdyn.client
import bosdyn.client.util
from bosdyn import geometry
from bosdyn.api import geometry_pb2, image_pb2, trajectory_pb2, world_object_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b,
                                         get_vision_tform_body)
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.lease import LeaseClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_id import RobotIdClient, version_tuple
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.network_compute_bridge import NetworkComputeBridgeClient

#pylint: disable=no-member
LOGGER = logging.getLogger()

# Use this length to make sure we're commanding the head of the robot
# to a position instead of the center.
BODY_LENGTH = 1.1

class Exit(object):
    """Handle exiting on SIGTERM."""

    def __init__(self):
        self._kill_now = False
        signal.signal(signal.SIGTERM, self._sigterm_handler)

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        return False

    def _sigterm_handler(self, _signum, _frame):
        self._kill_now = True

    @property
    def kill_now(self):
        """Return if sigterm received and program should end."""
        return self._kill_now

def get_input():
    """
    Get input from the user.
    """
    while True:
        try:
            x = int(input("Enter a number: "))
            break
        except ValueError:
            print("Invalid input. Try again.")
    return x

#------------------------------------------Detect Fiducial main function----------------------------------------------------------- 
#Call this function to start and detect board fiducials
# - Returns a set of fiducials when it finds the expected number of fiducials
def detectFiducial(self, int expectedNumberOfFiducials):
    int found = 0
    fiducial = set()
     while found != expectedNumberOfFiducials:
            if self._use_world_object_service:
                # Get the all fiducial objects
                fiducial = self.find_fiducials()
                if fiducial is not None:
                    found = len(fiducial)

            if detected_fiducial:
                print(fiducial)
            else:
                print('Trying to find fiducials...')

    return fiducial
     
#Find Fiducials and return a set of id numbers
def find_fiducials(self):
     #Get all fiducials that Spot detects with its perception system.
        # Get all fiducial objects (an object of a specific type).
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = self._world_object_client.list_world_objects(object_type=request_fiducials).world_objects
        if len(fiducial_objects) > 0:
            ids = set()
            # Find fiducials id
            for fiducial in fiducial_objects:
                if(fiducial.apriltag_properties.tag_id != 543) #Ignore fiducial id that represents the board
                {
                    ids.add(fiducial.apriltag_properties.tag_id)
                }
            #IMPORTANT, it sorts the list of IDS in order
            sorted_list = list(ids)
            sorted_list.sort()
            return sorted_list
        # Return none if no fiducials are found.
        return None

#needed for SPOT to detect fiducials 
def check_if_version_has_world_objects(self, robot_id):
        """Check that software version contains world object service."""
        # World object service was released in spot-sdk version 1.2.0
        return version_tuple(robot_id.software_release.version) >= (1, 2, 0)

 
def displayBoard():
    for i in range(3):
        for j in range(3):
            print(bi.boardState[i][j], end=" ")
        print()
 
def convertTo2DArray(markerIds):
    ret = []
    for i in range(len(markerIds)):
        ret.append(markerIds[i][0])
    return ret


# example python main.py -s tictactoe -m my_efficient_model -c 0.85 -d 0.5 --avoid-obstacles True

#==================================Main Function===================================================
def main():
    load_dotenv()
    username = os.getenv("USERNAME")
    password = os.getenv("PASSWORD")
#==================================Parse args===================================================
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('-s', '--ml-service',
                        help='Service name of external machine learning server.', required=True)
    parser.add_argument('-m', '--model', help='Model name running on the external server.',
                        required=True)
    parser.add_argument('-c', '--confidence-piece',
                        help='Minimum confidence to return an object for the dogoy (0.0 to 1.0)',
                        default=0.5, type=float)
    parser.add_argument('-d', '--distance-margin', default=.5,
                        help='Distance [meters] that the robot should stop from the fiducial.')
    parser.add_argument('--limit-speed', default=True, type=lambda x: (str(x).lower() == 'true'),
                        help='If the robot should limit its maximum speed.')
    parser.add_argument('--avoid-obstacles', default=False, type=lambda x:
                        (str(x).lower() == 'true'),
                        help='If the robot should have obstacle avoidance enabled.')
    parser.add_argument(
        '--use-world-objects', default=True, type=lambda x: (str(x).lower() == 'true'),
        help='If fiducials should be from the world object service or the apriltag library.')
    options = parser.parse_args()

# ===============================Start SPOT/Power On===========================================
    sdk = bosdyn.client.create_standard_sdk('TicTacSPOT')
    sdk.register_service_client(NetworkComputeBridgeClient)
    robot = sdk.create_robot(options.hostname)
    
    fiducial_follower = None
    image_viewer = None
    bosdyn.client.util.authenticate_with_client_certificate(robot, username, password)
    robot.time_sync.wait_for_sync()

    network_compute_client = robot.ensure_client(NetworkComputeBridgeClient.default_service_name)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    manipulate_api_client = robot.ensure_client(ManipulateApiClient.default_service_name)    

    
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        
        #Powering up robot
        robot.logger.info("Powering on robot... This may take a few seconds.")
        robot.power_on(timeout_sec=40)
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot is powered on.")

        #Spot Stand Up
        robot.logger.info("Commanding Spot to stand...")
        blocking_stand(command_client)
        time.sleep(.35)

        #------------------------------Initialize Values-------------------------------------
        #Put id values of board in this order:
        #       Ex: initial_values = [0,1,2,3,4,5,6,7,8]
        #          
        #     Represents a board with ids in this order...
        #           0 1 2
        #           3 4 5
        #           6 7 8
        initial_values = [0,1,2,3,4,5,6,7,8]

        #Player expected to be first
        board = bi.BoardInput()
        board.changeInitialState(initial_values)
        board.printBoard()
        int expectedNumberOfFiducials = 8
        player = ttt.O
        
        #Get Fiducials
        # while loop insert here <----------Game Loop starts

        #1. Find Fidicials and Update Board ----> Player move
        setOfIds = detectFiducial(expectedNumberOfFiducials)
        board.updateBoard(setOfIds, player)
        
        print("Detection done, found players move....")
        print("-----------------Board State:-------------")
        board.printBoard()
        board.addOPiece()
        board.updateTotalPieces()
        print("Player pieces:" + board.getOPieces)
        print("SPOT's Pieces: " + board.getXPieces)
        print("Total Pieces on board: " + board.getTotalPieces)
        print("------------------------------------------")

        #2. Minimax
        move = ttt.minimax(board.getBoardState())
        print(move)





 # ===============================Get Lease===========================================
            
    # elapsed_time = time.time() - start_time
    # interval = 3
    # while ttt.terminal(bi.boardState):
    #     # Update Board
    #     if elapsed_time > interval:
    #         detectArray = convertTo2DArray(markerIds)
    #         print("Input:")
    #         print(detectArray)
    #         bi.updateTotalPieces(bi)
    #         valid = bi.checkValidInput(detectArray)
            
    #         if (valid):
    #             if (player == ttt.X): #Players Turn                   
    #                 bi.updateBoard(detectArray,player,bi)
    #                 print("Player is")
    #                 print(player)
    #                 bi.totalXPieces += 1
    #         if (player == ttt.O): # AI Turn        
    #             print("AI Move: ")
    #             move = ttt.minimax(bi.boardState)
    #             print(move)
    #             bi.boardState = ttt.result(bi.boardState, move)

    #             bi.totalOPieces += 1
    #         displayBoard()
            
    #         if ttt.terminal(bi.boardState):
    #             print("Winner is")
    #             print(ttt.winner(bi.boardState))

            
            
    #         player = ttt.player(bi.boardState)  
    #         start_time = time.time()
if __name__ == '__main__':
    main()