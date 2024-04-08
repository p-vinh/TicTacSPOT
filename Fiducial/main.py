#MAIN GAMEPLAY LOOP
import argparse
import tictactoe as ttt
import boardInput as bi
import logging
from dotenv import load_dotenv
import time
import bosdyn.client
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.api import world_object_pb2, estop_pb2
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b,
                                         get_vision_tform_body)
from bosdyn.client.lease import LeaseClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_id import version_tuple
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.network_compute_bridge_client import NetworkComputeBridgeClient
from bosdyn.client.estop import EstopClient

import fetch_only_pickup as fetch
import fiducial_follow as follow
import arm_joint_move as place
import bosdyn.client.estop

#pylint: disable=no-member
LOGGER = logging.getLogger()
BOARD_REF = 535
LIST_IDS = [526, 527, 528, 529, 530,
            531, 532, 533,
            534]
# Use this length to make sure we're commanding the head of the robot
# to a position instead of the center.
BODY_LENGTH = 1.1


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

def detectFiducial(expectedNumberOfFiducials, pitch):
        
    # Clamp Pitch
    max_pitch = 0.45
    pitch = min(pitch, max_pitch)
    change_pitch(pitch)

    time.sleep(0.5)
    
    found = 0
    maxNumberOfIterations = 30
    fiducial = set()
    detect = False
    while found != expectedNumberOfFiducials and maxNumberOfIterations > 0:
        # Get the all fiducial objects
        fiducial = find_fiducials()
        if fiducial is not None:
            found = len(fiducial)
            detect = True
        if detect:
            print(fiducial, found, maxNumberOfIterations)
        maxNumberOfIterations-= 1
    
    if found == expectedNumberOfFiducials:
        return fiducial
    elif maxNumberOfIterations <= 0:
        pitch -= 0.1
        return detectFiducial(expectedNumberOfFiducials, pitch)

        
def change_pitch(pitch):
    footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=pitch)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    command_client.robot_command(cmd)
    robot.logger.info('Robot Pitch')
    
#Find Fiducials and return a set of id numbers
def find_fiducials():
     #Get all fiducials that Spot detects with its perception system.
        # Get all fiducial objects (an object of a specific type).
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = _world_object_client.list_world_objects(object_type=request_fiducials).world_objects
        if len(fiducial_objects) > 0:
            ids = {}
            # Find fiducials id
            for fiducial in fiducial_objects:
                if(fiducial.apriltag_properties.tag_id != BOARD_REF): #Ignore fiducial id that represents the board
                    tag_id = fiducial.apriltag_properties.tag_id
                    if tag_id not in ids:
                        # ID | World Position
                        ids[tag_id] = get_a_tform_b(fiducial.transforms_snapshot, VISION_FRAME_NAME, fiducial.apriltag_properties.frame_name_fiducial).to_proto()
            #IMPORTANT, it sorts the list of IDS in order
            sorted_list = sorted(ids.items(), key=lambda x: x[0])
            
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
    parser.add_argument('-d', '--distance-margin', default=0.6,
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

        #------------------------------Initialize Values-------------------------------------
        #Put id values of board in this order:
        #       Ex: initial_values = [0,1,2,3,4,5,6,7,8]
        #          
        #     Represents a board with ids in this order...
        #           0 1 2
        #           3 4 5
        #           6 7 8
        initial_values = LIST_IDS
        expectedNumberOfFiducials = 8
        player = ttt.O

        #Player expected to be first
        board = bi.BoardInput()
        board.changeInitialState(initial_values)
        board.printBoard()
        
        #Get Fiducials
        # while loop insert here <----------Game Loop starts

        #1. Find Fidicials and Update Board ----> Player move
        #Have Spot twist up to see all fiducials        
        idpos = detectFiducial(expectedNumberOfFiducials, -0.2) #list of id and postion (aka coord) pairs
        id_fid = [id[0] for id in idpos]
        print(id_fid)
        board.updateBoard(id_fid, player) #updates board
        
        print("Detection done, found players move....")
        print("-----------------Board State:-------------")
        board.printBoard()
        board.addOPiece()
        board.updateTotalPieces()
        print("Player pieces:", board.getOPieces())
        print("SPOT's Pieces: ", board.getXPieces())
        print("Total Pieces on board: ", board.getTotalPieces())
        print("------------------------------------------")

        #Have SPOT go back to stand position
        goBackToSame =  bosdyn.geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.0)
        cmd2 = RobotCommandBuilder.synchro_stand_command(footprint_R_body=goBackToSame)
        command_client.robot_command(cmd2)
        robot.logger.info('Robot is back to stand position')
        time.sleep(3)
        
        #2. Minimax
        move, id = ttt.minimax(board.getBoardState())
        
        #3. Pick Piece
        robot.logger.info('Sending Robot Pickup Request')
        fetch.pick_up(options, robot)
        time.sleep(1) # Wait for pickup to finish
        
        # 4. Set up Position
        print("Placing Piece....")
        class_obj = follow.fiducial_follow(robot, options, 535)
        
        # movePos.position.x movePos.rotation.x 
        movePos = idpos.get(id)
        # We want to tilt until we see the whole board:
        detectFiducial(expectedNumberOfFiducials, -0.2)
        print(movePos)
        # 5. Place Piece
        place.place_piece(robot, movePos)
        
        # 6. Backup From Reference Point
        # class_obj.backup_from_reference(2) # Backup 5 meters from reference point
        
        
        # 7. Gameover?
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
            # break
        
        # Wait for player to place their piece
        expectedNumberOfFiducials -= 1   
        time.sleep(10)         
            

if __name__ == '__main__':
    main()