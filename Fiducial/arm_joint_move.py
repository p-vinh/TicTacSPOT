# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Tutorial to show how to use Spot's arm.
"""
import argparse
import time
import fiducial_follow as follow

from bosdyn.api import geometry_pb2

import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.client import math_helpers
from bosdyn.geometry import EulerZXY
from bosdyn.api import (
    world_object_pb2,
)
from bosdyn.client.lease import LeaseClient
from bosdyn.client.frame_helpers import (
    VISION_FRAME_NAME,
    get_a_tform_b,
    BODY_FRAME_NAME,
    HAND_FRAME_NAME,
    get_vision_tform_body,
)
from bosdyn.client.world_object import WorldObjectClient

from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    block_until_arm_arrives,
    blocking_stand,
)
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.math_helpers import Quat
from bosdyn.client.robot_state import RobotStateClient
from dotenv import load_dotenv


# Using Claude 3.5 Sonnet
def detect_fiducial(world_object_client, fiducial_id):
    # Request the latest world objects (including fiducials)
    request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
    fiducial_objects = world_object_client.list_world_objects(object_type=request_fiducials).world_objects
    # Find the specified fiducial
    for obj in fiducial_objects:
        if obj.apriltag_properties and obj.apriltag_properties.tag_id == fiducial_id:
            # Get the transform from vision frame to fiducial
            vision_tform_fiducial = obj.transforms_snapshot.get_a_tform_b(
                VISION_FRAME_NAME, obj.apriltag_properties.frame_name_fiducial)
            
            if vision_tform_fiducial:
                return vision_tform_fiducial
    return None

def move_arm_to_fiducial(robot, command_client, fiducial_position):
    # Convert the fiducial position to a pose
    pose = math_helpers.SE3Pose(x=fiducial_position[0], y=fiducial_position[1], z=fiducial_position[2], 
                                rot=EulerZXY(yaw=0, pitch=0, roll=0))
    
    # Create the arm command
    arm_command = RobotCommandBuilder.arm_pose_command(
        pose.x, pose.y, pose.z, pose.rot.yaw, pose.rot.pitch, pose.rot.roll,
        frame_name=VISION_FRAME_NAME
    )
    
    # Send the command to the robot
    cmd_id = command_client.robot_command(arm_command)
    
    # Wait for the command to complete
    robot.time_sync.sleep_until(robot.time_sync.robot_timestamp_from_local_secs(time.time() + 5))

def place_piece(robot, command_client, world_object_client, fiducial_id):
    # Detect the fiducial
    vision_tform_fiducial = detect_fiducial(world_object_client, fiducial_id)
    
    if vision_tform_fiducial is None:
        print(f"Fiducial with ID {fiducial_id} not found.")
        return
    
    # Get the position of the fiducial in the vision frame
    fiducial_position = vision_tform_fiducial.get_translation()
    
    # Move the arm to a position slightly above the fiducial
    above_position = (fiducial_position[0], fiducial_position[1], fiducial_position[2] + 0.3)
    move_arm_to_fiducial(robot, command_client, above_position)
    
    # Move the arm down to make contact with the board
    move_arm_to_fiducial(robot, command_client, fiducial_position)
    
    # Open the gripper to release the piece
    gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
    command_client.robot_command(gripper_command)
    
    # Move the arm back up
    move_arm_to_fiducial(robot, command_client, above_position)

### chatgpt 4o / ali
# Get all detected fiducials 
# def get_fiducial_objects(world_object_client):
#     """Get all fiducial objects detected by Spot."""
#     request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
#     response = world_object_client.list_world_objects(object_type=request_fiducials)
#     return response.world_objects

# # Get a specific Fiducial ID
# def find_fiducial(world_object_client, fid_id):
#     fiducial_objects = get_fiducial_objects(world_object_client)

#     for fiducial in fiducial_objects:
#         if fiducial.apriltag_properties.tag_id == fid_id:
#             return fiducial
#     print(f"Fiducial with {fid_id} not found!")
#     return None

# def move_arm(robot, target_position, target_orientation, reference_frame, duration):
#     command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    
#     # Create the command to move the arm to the target position
#     arm_command = RobotCommandBuilder.arm_pose_command(
#         target_position.x, target_position.y, target_position.z,
#         target_orientation.w, target_orientation.x, target_orientation.y,
#         target_orientation.z, reference_frame, duration
#     )

#     # Send the command and wait for the arm to arrive
#     cmd_id = command_client.robot_command(arm_command)
#     block_until_arm_arrives(command_client, cmd_id)


# def place_piece(robot, fid_id):
#     robot.time_sync.wait_for_sync()
#     command_client = robot.ensure_client(RobotCommandClient.default_service_name)
#     robot_state = robot.ensure_client(RobotStateClient.default_service_name)
#     world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)


#     target_fiducial = find_fiducial(world_object_client, fid_id)
#     if not target_fiducial:
#         return
#     # Assuming SPOT is ready to place piece: standing aligned to the refrence fiducial, arm stowed with TicTacToe piece

#     vision_tform_fiducial = get_a_tform_b(
#         target_fiducial.transforms_snapshot, VISION_FRAME_NAME,
#         target_fiducial.apriltag_properties.frame_name_fiducial).to_proto()
#     if vision_tform_fiducial is not None:
#         fiducial_position = vision_tform_fiducial.position
#         fiducial_orientation = vision_tform_fiducial.orientation

#         # Calculate the intermediate approach position with fixed X
#         fixed_x = 0.5  # Adjust this value based on your environment for a safe approach
#         approach_position_yz = geometry_pb2.Vec3(
#             x=fixed_x,  # Fixed X position for safe approach
#             y=fiducial_position.y,
#             z=fiducial_position.z
#         )

#         # Calculate the final approach position
#         final_approach_position = geometry_pb2.Vec3(
#             x=fiducial_position.x,  # Move to the exact X position of the fiducial
#             y=fiducial_position.y,
#             z=fiducial_position.z
#         )

#         # Move the arm along Y and Z axes first with fixed X
#         move_arm(robot, approach_position_yz, fiducial_orientation, VISION_FRAME_NAME, 2.0)

#         # Finally, move the arm along the X axis to place the piece
#         move_arm(robot, final_approach_position, fiducial_orientation, VISION_FRAME_NAME, 2.0)


#         # Open the gripper to release the piece
#         gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
#         command = RobotCommandBuilder.build_synchro_command(gripper_command)            # build_synchro_command may be unnecessary
#         cmd_id = command_client.robot_command(command)
#         block_until_arm_arrives(command_client, cmd_id)
        
#         time.sleep(1)

#         # Stow the arm
#         # Build the stow command using RobotCommandBuilder
#         stow = RobotCommandBuilder.arm_stow_command()

#         # Issue the command via the RobotCommandClient
#         stow_command_id = command_client.robot_command(stow)

#         robot.logger.info('Stow command issued.')
#         block_until_arm_arrives(command_client, stow_command_id, 3.0)
        
#         print("Piece placed successfully!")
    


###

###
#by Deyi, this might solve the placement issue, and this function will be integate in place_piece function
def control_gripper(command_client, open_fraction):
    gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(open_fraction)
    command = RobotCommandBuilder.build_synchro_command(gripper_command)
    cmd_id = command_client.robot_command(command)
    block_until_arm_arrives(command_client, cmd_id, 5.0)

def place_piece(robot, fid_id):
    robot.time_sync.wait_for_sync()
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    robot_state = robot.ensure_client(RobotStateClient.default_service_name)

    def get_fiducial_objects():
        """Get all fiducials that Spot detects with its perception system."""
        # Get all fiducial objects (an object of a specific type).

        _world_object_client = robot.ensure_client(
            WorldObjectClient.default_service_name
        )
        
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = _world_object_client.list_world_objects(
            object_type=request_fiducials
        ).world_objects
        if len(fiducial_objects) > 0:
            # Return all fiducial objects it sees
            for fid in fiducial_objects:
                if fid.apriltag_properties.tag_id == fid_id:
                    return fid
        return None

    robot_state = robot.ensure_client(RobotStateClient.default_service_name)
    fiducial = get_fiducial_objects()
    arm_command = None
        
    if fiducial is not None:
        vision_tform_fiducial = get_a_tform_b(
            fiducial.transforms_snapshot,
            VISION_FRAME_NAME,
            fiducial.apriltag_properties.frame_name_fiducial,
        ).to_proto()

        body_control = spot_command_pb2.BodyControlParams(
            body_assist_for_manipulation=spot_command_pb2.BodyControlParams.
            BodyAssistForManipulation(enable_hip_height_assist=True, enable_body_yaw_assist=True))
        body_assist_enabled_stand_command = RobotCommandBuilder.synchro_stand_command(
            params=spot_command_pb2.MobilityParams(body_control=body_control))

        robot_rt_world = get_vision_tform_body(robot_state.get_robot_state().kinematic_state.transforms_snapshot)

        
        # Unstow the arm
        ready_command = RobotCommandBuilder.arm_ready_command(
            build_on_command=body_assist_enabled_stand_command)
        ready_command_id = command_client.robot_command(ready_command)
        robot.logger.info('Going to "ready" pose')
        block_until_arm_arrives(command_client, ready_command_id, 3.0)
        
        print ("VISION_TFORM_FIDUCIAL: ")
        print(vision_tform_fiducial)
        # print(rotation)
        print ("ROBOT_RT_WORLD: ")
        print(robot_rt_world)
                    
        rotation = Quat()
        # raise_arm = RobotCommandBuilder.arm_pose_command(
        #     1,
        #     vision_tform_fiducial.position.y - robot_rt_world.position.y,
        #     vision_tform_fiducial.position.z - robot_rt_world.position.z,
        #     rotation.w,
        #     rotation.x,
        #     rotation.y,
        #     rotation.z,
        #     frame_name=BODY_FRAME_NAME,
        # )
        
        # print ("Doing raise arm command...")
        # command = RobotCommandBuilder.build_synchro_command(raise_arm)
        # cmd_id = command_client.robot_command(command)
        # block_until_arm_arrives(command_client, cmd_id)
        
        arm_command = RobotCommandBuilder.arm_pose_command_from_pose(vision_tform_fiducial, VISION_FRAME_NAME, seconds=7, build_on_command=body_assist_enabled_stand_command)
        # gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
        
        #Move SPOT arm relative to fiducial position
        # command = RobotCommandBuilder.build_synchro_command(
        #     gripper_command, arm_command)
        
        print ("Doing arm pose command....")
        cmd_id = command_client.robot_command(arm_command)
        
        block_until_arm_arrives(command_client, cmd_id)

        # Open gripper to release the piece
        print ("Opening gripper... ")
        control_gripper(command_client, open_fraction=1.0) 
        
        # Make the open gripper RobotCommand
        # gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(12.0)

        # # Combine the arm and gripper commands into one RobotCommand
        # command = RobotCommandBuilder.build_synchro_command(gripper_command)
        # cmd_id = command_client.robot_command(command)
        
    print("Carrying Finished, Stowing...")
    stow = RobotCommandBuilder.arm_stow_command()
    block_until_arm_arrives
    (
        command_client, command_client.robot_command(stow), 3.0
    )


# Testing function
if __name__ == "__main__":
    load_dotenv()
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('-d', '--distance-margin', default=0.75,
                        help='Distance [meters] that the robot should stop from the fiducial.')
    parser.add_argument('--limit-speed', default=True, type=lambda x: (str(x).lower() == 'true'),
                        help='If the robot should limit its maximum speed.')
    parser.add_argument('--avoid-obstacles', default=False, type=lambda x:
                        (str(x).lower() == 'true'),
                        help='If the robot should have obstacle avoidance enabled.')
    
    options = parser.parse_args()

    sdk = bosdyn.client.create_standard_sdk("TicTacSPOT")
    robot = sdk.create_robot("192.168.80.3")
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    lease_client.take()
    with bosdyn.client.lease.LeaseKeepAlive(
        lease_client, must_acquire=True, return_at_exit=True
    ):
        robot.logger.info("Powering on robot... This may take a few seconds.")
        robot.power_on(timeout_sec=40)
        
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot is powered on.")

        assert robot.has_arm(), "Robot requires an arm to run."

        blocking_stand(command_client)
        time.sleep(0.35)
        
        class_obj = follow.fiducial_follow(robot, options, 536)
        
        def change_pitch(pitch):
            footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=pitch)
            cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
            command_client.robot_command(cmd)
            robot.logger.info('Robot Pitch')
        
        change_pitch(-0.45)
        time.sleep(2)
        place_piece(robot, 537)