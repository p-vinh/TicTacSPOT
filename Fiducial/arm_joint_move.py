# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).


"""
Tutorial to show how to use Spot's arm to detect and interact with fiducial markers (AprilTags).
This script is a modified version of the fiducial_follow example from Boston Dynamics SDK. Specifically, boston-dynamics/python/examples/fiducial_follow 
https://github.com/boston-dynamics/spot-sdk/tree/master/python/examples/fiducial_follow
"""
import argparse
import time
import fiducial_follow as follow

import numpy as np

from bosdyn.api import geometry_pb2, arm_command_pb2, synchronized_command_pb2


import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
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



# Modified code from fiducial_follow in Boston Dynamics SDK python/examples
# utilized ChatGPT to fix some errors
def detect_fiducial(world_object_client, fiducial_id):
    # Request the latest world objects (including fiducials)
    request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
    fiducial_objects = world_object_client.list_world_objects(object_type=request_fiducials).world_objects
 
    # Find the specified fiducial
    for fid in fiducial_objects:
        if fid.apriltag_properties and fid.apriltag_properties.tag_id == fiducial_id:
            # Return the fiducial frame name
            return fid
    return None


def offset_tag_pose(robot_state, object_rt_world, dist_margin=1.0):
    """Offset the go-to location of the fiducial and compute the desired heading."""
    robot_rt_world = get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)
    # This works but maybe also making the calcuation in the Z axis will improve it too
    robot_to_object_ewrt_world = np.array(
        [object_rt_world.x - robot_rt_world.x, object_rt_world.y - robot_rt_world.y, 0])
    # Test whether changing the Z axis here improves the placement accuracy
    #robot_to_object_ewrt_world = np.array(
    #    [object_rt_world.x - robot_rt_world.x, object_rt_world.y - robot_rt_world.y, object_rt_world.z - robot_rt_world.z])
    robot_to_object_ewrt_world_norm = robot_to_object_ewrt_world / np.linalg.norm(
        robot_to_object_ewrt_world)
    heading = get_desired_angle(robot_to_object_ewrt_world_norm)
    goto_rt_world = np.array([
        object_rt_world.x - robot_to_object_ewrt_world_norm[0] * dist_margin,
        object_rt_world.y - robot_to_object_ewrt_world_norm[1] * dist_margin,
        object_rt_world.z - robot_to_object_ewrt_world_norm[2] * dist_margin

    ])
    return goto_rt_world, heading


def get_desired_angle(xhat):
    """Compute heading based on the vector from robot to object."""
    zhat = [0.0, 0.0, 1.0]
    yhat = np.cross(zhat, xhat)
    mat = np.array([xhat, yhat, zhat]).transpose()
    return Quat.from_matrix(mat).to_yaw()

#by Deyi
def control_gripper(command_client, open_fraction):
    gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(open_fraction)
    command = RobotCommandBuilder.build_synchro_command(gripper_command)
    cmd_id = command_client.robot_command(command)
    block_until_arm_arrives(command_client, cmd_id, 5.0)
   
def place_piece(robot, fiducial_id):
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
   
    # Unstow (ready) the arm
    unstow_command = RobotCommandBuilder.arm_ready_command()
    # Send the command and block until the arm reaches the ready position
    block_until_arm_arrives(command_client, command_client.robot_command(unstow_command), 3.0)
    time.sleep(2)


    # Detect Fiducial and its frame name
    fiducial = detect_fiducial(world_object_client, fiducial_id)
    if fiducial is not None:
        vision_tform_fiducial = get_a_tform_b(
            fiducial.transforms_snapshot, VISION_FRAME_NAME,
            fiducial.apriltag_properties.frame_name_fiducial).to_proto()
        if vision_tform_fiducial is not None:
            fiducial_rt_world = vision_tform_fiducial.position
        else:
            print(f"Fiducial with ID {fiducial_id} not found.")
            return
    else:
        print(f"Fiducial with ID {fiducial_id} not found.")
        return


    print(f"Fiducial ID: {fiducial.apriltag_properties.tag_id}")
    print(f"Fiducial apriltag: {fiducial.apriltag_properties}")


    # Get the robot state
    robot_state = robot_state_client.get_robot_state()


    # Define the position and orientation for the arm to move to
    current_tag_world_pose, angle_desired = offset_tag_pose(robot_state, fiducial_rt_world, .01)
   
    initial_offset = 0.5  # Adjust this offset value as needed
    initial_x_position = current_tag_world_pose[0] - initial_offset  # Move slightly forward of the fiducial

    arm_pose_command = RobotCommandBuilder.arm_pose_command(
        x=initial_x_position,
        y=current_tag_world_pose[1],
        z=current_tag_world_pose[2],
        qw=np.cos(angle_desired / 2),
        qx=0.0,
        qy=0.0,
        qz=np.sin(angle_desired / 2),
        frame_name=VISION_FRAME_NAME,
        seconds=3  # Duration to achieve the pose
    )


    # Send the command to the robot
    cmd_id = command_client.robot_command(arm_pose_command)
    block_until_arm_arrives(command_client, cmd_id, 10.0)  

    # Build the arm pose command
    arm_pose_command = RobotCommandBuilder.arm_pose_command(
        x=current_tag_world_pose[0],
        y=current_tag_world_pose[1],
        z=current_tag_world_pose[2],
        qw=np.cos(angle_desired / 2),
        qx=0.0,
        qy=0.0,
        qz=np.sin(angle_desired / 2),
        frame_name=VISION_FRAME_NAME,
        seconds=3  # Duration to achieve the pose
    )


    # Send the command to the robot
    cmd_id = command_client.robot_command(arm_pose_command)
    block_until_arm_arrives(command_client, cmd_id, 10.0)  

    print(f"Placing piece on fiducial {fiducial_id}")

    # Open the gripper to release the piece
    control_gripper(command_client, 1)

    print("Stow")
    stow_command = RobotCommandBuilder.arm_stow_command()
    block_until_arm_arrives(command_client, command_client.robot_command(stow_command), 3.0)

    # Close gripper    
    control_gripper(command_client, 0)


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