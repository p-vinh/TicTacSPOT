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
def detectFiducial(self):
     #Get all fiducials that Spot detects with its perception system.
        # Get all fiducial objects (an object of a specific type).
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = self._world_object_client.list_world_objects(object_type=request_fiducials).world_objects
        if len(fiducial_objects) > 0:
            ids = set()
            # Return the first detected fiducial id.
            for fiducial in fiducial_objects:
                ids.add(fiducial.apriltag_properties.tag_id)
            return ids
        # Return none if no fiducials are found.
        return None
   
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
        robot.logger.info("Powering on robot... This may take a few seconds.")
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot is powered on.")

        robot.logger.info("Commanding Spot to stand...")
        blocking_stand(command_client)


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