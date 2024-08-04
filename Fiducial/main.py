#MAIN GAMEPLAY LOOP
import argparse
import tictactoe as ttt
import boardInput as bi
import goToInitial as goTo
import logging
from dotenv import load_dotenv
import time
import numpy as np

import bosdyn.client
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.api import world_object_pb2, estop_pb2
from bosdyn.client.frame_helpers import (GRAV_ALIGNED_BODY_FRAME_NAME,BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b,
                                         get_vision_tform_body)
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.client.lease import LeaseClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_id import version_tuple
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.network_compute_bridge_client import NetworkComputeBridgeClient
from bosdyn.client.estop import EstopClient
from bosdyn.client.robot_state import RobotStateClient

import fetch_only_pickup as fetch
import fiducial_follow as follow
import arm_joint_move as place
import bosdyn.client.estop

#pylint: disable=no-member
LOGGER = logging.getLogger()
BOARD_REF = 549
LIST_IDS = [530, 531, 532, 533, 534, 535, 536, 537, 538]
# Use this length to make sure we're commanding the head of the robot
# to a position instead of the center.
BODY_LENGTH = 1.1


def verify_estop(robot):
    """Verify the robot is not estopped"""

    client = robot.ensure_client(EstopClient.default_service_name)
    if client.get_status().stop_level != estop_pb2.ESTOP_LEVEL_NONE:
        error_message = 'Robot is estopped. Please use an external E-Stop client, such as the ' \
                        'estop SDK example, to configure E-Stop.'
        robot.logger.error(error_message)
        raise Exception(error_message)

#------------------------------------------Detect Fiducial main function----------------------------------------------------------- 
#Call this function to start and detect board fiducials
# - Returns a set of fiducials when it finds the expected number of fiducials

def detectFiducial(expectedNumberOfFiducials, pitch, reset_pitch = 0):
    # Clamp Pitch
    pitch = min(pitch, 0.45)

    # Base case to prevent infinite recursion
    if pitch < -0.45:
        print("Could not detect expected # of fiducials!")
        print(f"Pitch reached: {pitch}")
        print(f"Resetting pitch to {reset_pitch}")
        change_pitch(reset_pitch)
        return None
    
    change_pitch(pitch)
    time.sleep(0.5)
    
    for i in range(30):
        fiducials = find_fiducials()
        if fiducials and len(fiducials) == expectedNumberOfFiducials:
            print(fiducials, len(fiducials), 30 - i)
            print(f"Resetting pitch to {reset_pitch}")
            change_pitch(reset_pitch)
            return fiducials
        time.sleep(0.1)
    
    return detectFiducial(expectedNumberOfFiducials, pitch - 0.1)
    
    

        
def change_pitch(pitch):
    footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=pitch)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    command_client.robot_command(cmd)
    robot.logger.info('Robot Pitch')
    
#Find Fiducials and return a set of id numbers
def find_fiducials():
    """
    Retrieve all fiducials detected by Spot's perception system,
    excluding the fiducial that represents the board.
    """

    # Request fiducials of type AprilTags
    request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]

    fiducial_objects = _world_object_client.list_world_objects(object_type=request_fiducials).world_objects
    if len(fiducial_objects) > 0:
        ids = set()
        # Find fiducials id
        for fiducial in fiducial_objects:
            if(fiducial.apriltag_properties.tag_id != BOARD_REF): #Ignore fiducial id that represents the board
                ids.add(fiducial.apriltag_properties.tag_id)
        #IMPORTANT, it sorts the list of IDS in order
        return sorted(ids)

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
#==================================Parse args===================================================
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('-s', '--ml-service',
                        help='Service name of external machine learning server.', required=True)
    parser.add_argument('-m', '--model', help='Model name running on the external server.',
                        required=True)
    parser.add_argument('-c', '--confidence-piece',
                        help='Minimum confidence to return an object for the dogoy (0.0 to 1.0)',
                        default=0.8, type=float)
    parser.add_argument('-d', '--distance-margin', default=0.60,
                        help='Distance [meters] that the robot should stop from the fiducial.')
    parser.add_argument('--limit-speed', default=True, type=lambda x: (str(x).lower() == 'true'),
                        help='If the robot should limit its maximum speed.')
    parser.add_argument('--avoid-obstacles', default=False, type=lambda x:
                        (str(x).lower() == 'true'),
                        help='If the robot should have obstacle avoidance enabled.')
    
    options = parser.parse_args()

# ===============================Start SPOT/Power On===========================================
    global _world_object_client
    global lease_client
    global command_client
    global robot
    sdk = bosdyn.client.create_standard_sdk('TicTacSPOT')
    sdk.register_service_client(NetworkComputeBridgeClient)
    robot = sdk.create_robot(options.hostname)

    
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    _world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    _robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    
    lease_client.take()
    # ===============================Get Lease===========================================
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        
        #Powering up robot
        robot.logger.info("Powering on robot... This may take a few seconds.")
        robot.power_on(timeout_sec=40)
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot is powered on.")
        
        assert robot.has_arm(), 'Robot requires an arm to run.'
        verify_estop(robot)

        #Spot Stand Up
        robot.logger.info("Commanding Spot to stand...")
        blocking_stand(command_client)
        time.sleep(.35)

        #perhaps raise Spot taller to increase perception of the board
        # height = .2
        # cmd = RobotCommandBuilder.synchro_stand_command(body_height=height)
        # command_client.robot_command(cmd)
        # robot.logger.info('Set body height to {}'.format(height))

        expectedNumberOfFiducials = 9
        playerTurn = ttt.O
        spotTurn = ttt.X
        board = bi.BoardInput(LIST_IDS)
        board.printBoard()
        
         
        #Obtain initial coordinates - these hold SPOTS initial position when booting up
        robot_initial_coords = get_vision_tform_body(_robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
        print("Robot Initial Coords:")
        print(robot_initial_coords.position)
        
        # 1. Find Fidicials and Update Board 
        # Assuming player initially placed an O piece on the board (player went when there was 9 (odd number) open fiducials)
        print("Player's turn!")
        expectedNumberOfFiducials -= 1
        ids = detectFiducial(expectedNumberOfFiducials, -0.2)
        board.updateBoard(ids, playerTurn)

        board.printBoardInfo()


        # Continue game until board is filled up
        while(expectedNumberOfFiducials > 0):
            # Deciding who's turn
            if(expectedNumberOfFiducials % 2 == 0):
                currentTurn = spotTurn
            else:
                currentTurn = playerTurn

            if(currentTurn == playerTurn):
                print("Player's turn!")
                time.sleep(15) # Wait for player to place piece
                expectedNumberOfFiducials -= 1
                ids = detectFiducial(expectedNumberOfFiducials, -0.2)
                board.updateBoard(ids, playerTurn)

                board.printBoardInfo()
            else:
                # currentTurn is Spots!
                print("Spot's turn!")
                # Calculate move
                move, id = ttt.minimax(board.getBoardState())

                # 3. Pick up piece
                robot.logger.info('Sending Robot Pickup Request')
                fetch.pick_up(options, robot)
                time.sleep(1) # Wait for pickup to finish
                print(move, id)
            
                # 4. Orient to the board using refrence fiducial
                print("Orienting to the board.")
                class_obj = follow.fiducial_follow(robot, options, BOARD_REF)

                # 5. Place piece
                detectFiducial(expectedNumberOfFiducials, -0.2)     # Perhaps delete this as I reset pitch anyways? Can test with/without to see what's better, maybe remove the resetpitch within the function and do it selectively so that we can take advantage of the detectFiducial function here to put us in a position where we see the desired fiducials

                place.place_piece(robot, id)

                expectedNumberOfFiducials -= 1
                ids = detectFiducial(expectedNumberOfFiducials, -0.2)
                board.updateBoard(ids, playerTurn)

                board.printBoardInfo()

                # 6. Orient itself back to the refrence point
                # Assuming when it placed its piece, it's too close to the board, so perhaps slowly back up some distance and than orient itself to the refrence.

                backup(1.5)
                # ^ create some function, now refrence fiducial should be in frame, lets go to the correct amount of distance to it?
                # like this?
                class_obj = follow.fiducial_follow(robot, options, BOARD_REF)
                class_obj.backup_from_reference(1.5) 
        # Expected number of fiducials is 0
        piece = ttt.winner(board.getBoardState())
        if piece == ttt.X:
            print("Spot wins")
            # DANCE
            # break For infinite game loop
        elif piece == ttt.O:
            print("Player wins")
            # break
        elif piece == None:
            print("No one won yet")
            
            







#old code
        # while(expectedNumberOfFiducials > 0): 
        #     # 1. Find Fidicials and Update Board ----> Player move
        #     # Have Spot twist up to see all fiducials        
        #     ids = detectFiducial(expectedNumberOfFiducials, -0.2) #list of id and postion (aka coord) pairs
            
        #     # Check turn, assuming player with piece O goes first
        #     if(expectedNumberOfFiducials % 2 == 0):
        #         turn = playerTurn
        #     else:
        #         t = spotTurn
        #     board.updateBoard(ids, move) 
            
        #     print("Detection done, found players move....")
        #     print("-----------------Board State:-------------")
        #     board.printBoard()
        #     board.addOPiece()
        #     print("Player pieces:", board.getOPieces())
        #     print("SPOT's Pieces: ", board.getXPieces())
        #     print("Total Pieces on board: ", board.getTotalPieces())
        #     print("------------------------------------------")

        #     # Have SPOT go back to stand position
        #     goBackToSame =  bosdyn.geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.0)
        #     cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=goBackToSame)
        #     command_client.robot_command(cmd)
        #     robot.logger.info('Robot is back to stand position')
        #     time.sleep(3)
            
        #     # #2. Minimax
        #     move, id = ttt.minimax(board.getBoardState())
            
        #     #3. Pick Piece
        #     robot.logger.info('Sending Robot Pickup Request')
        #     fetch.pick_up(options, robot)
        #     time.sleep(1) # Wait for pickup to finish
        #     print(move, id)
            
        #     # 4. Set up Position
        #     print("Placing Piece....")
            
        #     # Go back to original position
        #     # obj = goTo.headToNewCoords(robot, options, robot_initial_coords)
            
        #     time.sleep(1)
            
        #     #Go to Fiducial
        #     class_obj = follow.fiducial_follow(robot, options, BOARD_REF)
            
        #     # We want to tilt until we see the whole board:
            
        #     detectFiducial(expectedNumberOfFiducials, -0.2)
            
        #     # 5. Place Piece
        #     place.place_piece(robot, id)
            
        #     # 6. Backup From Reference Point
        #     class_obj.backup_from_reference(1.5) # Backup 1.1 meters from reference point
            
            
        # # All board fiducial pieces covered on:
        
        # piece = ttt.winner(board.getBoardState())
        # if piece == ttt.X:
        #     print("Spot wins")
        #     # DANCE
        #     # break For infinite game loop
        # elif piece == ttt.O:
        #     print("Player wins")
        #     # break
        # elif piece == None:
        #     print("No one won yet")
        #     # break
            
        #     # Wait for player to place their piece
        #     #might need to addPiece after this, but then question is how does the board keep track of it
            
        #                 # placedpieces = [531(O)]       

        #     #Spot placed piece, now append to the list
        #     player = ttt.X
        #     board.updateBoard(ids, player) #updates board

        #     #Now player puts his piece, append to the list    
        #     time.sleep(10)    #Waits for player for 10s, will change later to get player input instead
        #     player = ttt.O
        #     board.updateBoard(ids, player) #updates board
            
        #     expectedNumberOfFiducials -= 2
         
            

if __name__ == '__main__':
    main()