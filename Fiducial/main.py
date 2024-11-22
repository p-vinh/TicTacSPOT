#MAIN GAMEPLAY LOOP
import argparse
import tictactoe as ttt
import boardInput as bi
import goToInitial as goTo
import logging
from dotenv import load_dotenv
import time

import bosdyn.client
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.api import world_object_pb2, estop_pb2, geometry_pb2, trajectory_pb2
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME,
                                         get_se2_a_tform_b, get_vision_tform_body)
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.client.lease import LeaseClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_id import version_tuple
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.network_compute_bridge_client import NetworkComputeBridgeClient
from bosdyn.client.estop import EstopClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2


import fetch_only_pickup as fetch
import fiducial_follow as follow
import fiducial_follow_BostonDynamics as follow_BostonDynamics
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

def get_robot_coordinates(robot_state_client):
    """Returns the current coordinates of Spot in the world frame."""
    robot_state = get_vision_tform_body(robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
    return {
        'x': robot_state.x,
        'y': robot_state.y,
        'z': robot_state.z,
        'yaw': robot_state.rot.to_yaw()
    }

def go_to_coordinates(robot_command_client, x, y, yaw=0.0, frame_name='vision', body_height=0.0, locomotion_hint=spot_command_pb2.HINT_AUTO, powered_on=True):
    """Commands Spot to go to the given coordinates."""
    mobility_params = set_mobility_params()
    _robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=x, goal_y=y, goal_heading=yaw, frame_name=frame_name, params=mobility_params,
        body_height=body_height, locomotion_hint=locomotion_hint)
   
    end_time = 10.0  # Command duration in seconds
    if powered_on:
        # Issue the command to the robot
        robot_command_client.robot_command(lease=None, command=cmd,
                                           end_time_secs=time.time() + end_time)
        # Feedback loop to check and wait until the robot reaches the desired position or timeout
        start_time = time.time()
        current_time = time.time()
        while (not is_at_target(_robot_state_client, x, y, yaw) and current_time - start_time < end_time):
            time.sleep(.25)
            current_time = time.time()


def is_at_target(robot_state_client, x, y, yaw, epsilon=0.1):
    """Check if the robot is at the target position within a margin of error."""
    current_state = get_vision_tform_body(robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
    current_angle = current_state.rot.to_yaw()
    return (abs(current_state.x - x) < epsilon and
            abs(current_state.y - y) < epsilon and
            abs(current_angle - yaw) < 0.075)  # Adjust epsilon for yaw as needed


def set_mobility_params():
    """Set robot mobility params to disable obstacle avoidance."""
    obstacles = spot_command_pb2.ObstacleParams(disable_vision_body_obstacle_avoidance=True,
                                                disable_vision_foot_obstacle_avoidance=True,
                                                disable_vision_foot_constraint_avoidance=True,
                                                obstacle_avoidance_padding=.001)
    # Default body control settings
    body_control = set_default_body_control()
    speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(
        linear=Vec2(x=0.5, y=0.5), angular=1.0))
   
    mobility_params = spot_command_pb2.MobilityParams(
        obstacle_params=obstacles, vel_limit=speed_limit, body_control=body_control,
        locomotion_hint=spot_command_pb2.HINT_AUTO)
   
    return mobility_params


def set_default_body_control():
    """Set default body control params to current body position."""
    footprint_R_body = bosdyn.geometry.EulerZXY()
    position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
    rotation = footprint_R_body.to_quaternion()
    pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
    point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
    traj = trajectory_pb2.SE3Trajectory(points=[point])
    return spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

def orient_to_board(initial_coords, options):
    print("Orienting to the board.")

    print("Step 1: Going to initial boot up coordinates")
    go_to_coordinates(command_client, x=initial_coords['x'], y=initial_coords['y'], yaw=initial_coords['yaw'], powered_on=robot.is_powered_on)
    
    print("Step 2: Finding and going to board reference fiducial")
    class_obj = follow.fiducial_follow(robot, options, BOARD_REF)

    
    end_time = 10.0  # Command duration in seconds
    if powered_on:
        # Issue the command to the robot
        robot_command_client.robot_command(lease=None, command=cmd,
                                           end_time_secs=time.time() + end_time)
        # Feedback loop to check and wait until the robot reaches the desired position or timeout
        start_time = time.time()
        current_time = time.time()
        while (not is_at_target(_robot_state_client, x, y, yaw) and current_time - start_time < end_time):
            time.sleep(.25)
            current_time = time.time()

def is_at_target(robot_state_client, x, y, yaw, epsilon=0.1):
    """Check if the robot is at the target position within a margin of error."""
    current_state = get_vision_tform_body(robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
    current_angle = current_state.rot.to_yaw()
    return (abs(current_state.x - x) < epsilon and
            abs(current_state.y - y) < epsilon and
            abs(current_angle - yaw) < 0.075)  # Adjust epsilon for yaw as needed

def set_mobility_params():
    """Set robot mobility params to disable obstacle avoidance."""
    obstacles = spot_command_pb2.ObstacleParams(disable_vision_body_obstacle_avoidance=True,
                                                disable_vision_foot_obstacle_avoidance=True,
                                                disable_vision_foot_constraint_avoidance=True,
                                                obstacle_avoidance_padding=.001)
    # Default body control settings
    body_control = set_default_body_control()
    speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(
        linear=Vec2(x=0.5, y=0.5), angular=1.0))
    
    mobility_params = spot_command_pb2.MobilityParams(
        obstacle_params=obstacles, vel_limit=speed_limit, body_control=body_control,
        locomotion_hint=spot_command_pb2.HINT_AUTO)
    
    return mobility_params

def set_default_body_control():
    """Set default body control params to current body position."""
    footprint_R_body = bosdyn.geometry.EulerZXY()
    position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
    rotation = footprint_R_body.to_quaternion()
    pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
    point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
    traj = trajectory_pb2.SE3Trajectory(points=[point])
    return spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)


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

        # Spot Stand Up
        robot.logger.info("Commanding Spot to stand...")
        blocking_stand(command_client)
        time.sleep(.35)

        expectedNumberOfFiducials = 9
        playerTurn = ttt.O
        spotTurn = ttt.X
        board = bi.BoardInput(LIST_IDS)
        board.printBoard()
        
        # Record initial coordinates (for better gameplay, boot up Spot in an optimal position from the Tic-Tac-Toe board)
        initial_coords = get_robot_coordinates(_robot_state_client)
        print(f"Did new?Robot Initial Coordinates: {initial_coords}")

        # Find Fidicials and Update Board 
        # Assuming player initially placed an O piece on the board (player went when there was 9 (odd number) open fiducials)
        print("Player's turn! Assuming player has already placed O piece on board")
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
                orient_to_board(initial_coords, options)


                # 5. Place piece
                place.place_piece(robot, id)

                expectedNumberOfFiducials -= 1
                ids = detectFiducial(expectedNumberOfFiducials, -0.2)
                board.updateBoard(ids, playerTurn)

                board.printBoardInfo()

                # 6. Orient itself back to board
                orient_to_board(initial_coords, options)
            if(expectedNumberOfFiducials <= 4):    
                piece = ttt.winner(board.getBoardState())
                if piece == ttt.X:
                    print("Spot wins")
                    break
                elif piece == ttt.O:
                    print("Player wins")
                    break
        # Expected number of fiducials is 0, the fiducials are covered with pieces
        if piece == None:
            print("Draw!")
         
            

if __name__ == '__main__':
    main()