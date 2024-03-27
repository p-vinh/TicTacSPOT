# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Tutorial to show how to use Spot's arm.
"""
import argparse
import sys
import time

from google.protobuf import wrappers_pb2

import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import arm_command_pb2, robot_command_pb2, synchronized_command_pb2
from bosdyn.client.lease import LeaseClient

from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.util import duration_to_seconds

from dotenv import load_dotenv


def make_robot_command(arm_joint_traj):
    """ Helper function to create a RobotCommand from an ArmJointTrajectory.
        The returned command will be a SynchronizedCommand with an ArmJointMoveCommand
        filled out to follow the passed in trajectory. """

    joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(trajectory=arm_joint_traj)
    arm_command = arm_command_pb2.ArmCommand.Request(arm_joint_move_command=joint_move_command)
    sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
    arm_sync_robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=sync_arm)
    return RobotCommandBuilder.build_synchro_command(arm_sync_robot_cmd)


def print_feedback(feedback_resp, logger):
    """ Helper function to query for ArmJointMove feedback, and print it to the console.
        Returns the time_to_goal value reported in the feedback """
    joint_move_feedback = feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_joint_move_feedback
    logger.info(f'  planner_status = {joint_move_feedback.planner_status}')
    logger.info(
        f'  time_to_goal = {duration_to_seconds(joint_move_feedback.time_to_goal):.2f} seconds.')

    # Query planned_points to determine target pose of arm
    logger.info('  planned_points:')
    for idx, points in enumerate(joint_move_feedback.planned_points):
        pos = points.position
        pos_str = f'sh0 = {pos.sh0.value:.3f}, sh1 = {pos.sh1.value:.3f}, el0 = {pos.el0.value:.3f}, ' \
                  f'el1 = {pos.el1.value:.3f}, wr0 = {pos.wr0.value:.3f}, wr1 = {pos.wr1.value:.3f}'
        logger.info(f'    {idx}: {pos_str}')
    return duration_to_seconds(joint_move_feedback.time_to_goal)

def block_until_arm_arrives_with_prints(robot, command_client, cmd_id):
    """Block until the arm arrives at the goal and print the distance remaining.
        Note: a version of this function is available as a helper in robot_command
        without the prints.
    """
    while True:
        feedback_resp = command_client.robot_command_feedback(cmd_id)
        measured_pos_distance_to_goal = feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.measured_pos_distance_to_goal
        measured_rot_distance_to_goal = feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.measured_rot_distance_to_goal
        robot.logger.info('Distance to go: %.2f meters, %.2f radians',
                          measured_pos_distance_to_goal, measured_rot_distance_to_goal)

        if feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.status == arm_command_pb2.ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_COMPLETE:
            robot.logger.info('Move complete.')
            break
        time.sleep(0.1)

def joint_move_example(robot, move, id, command_client):
    """A simple example of using the Boston Dynamics API to command Spot's arm to perform joint moves."""
    # parser = argparse.ArgmentParser()
    # bosdyn.client.util.add_base_arguments(parser)
    # options = parser.parse_arg()
    
    try:
        if move[0] == 2:
            cmd_id = bottomRow(robot, command_client)
        elif move[0] == 1:
            cmd_id = middleRow(robot, command_client)
            #  stow(robot,command_client)
        elif move[0] == 0:
            cmd_id = topRow(robot, command_client)
        else:
             print("cannot do this....")
        
        # Make the open gripper RobotCommand
        # gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
        
        # Combine the arm and gripper commands into one RobotCommand
        # command = RobotCommandBuilder.build_synchro_command(gripper_command, gripper_command)
        # Send the request
        # cmd_id = command_client.robot_command(command)
        
        # Wait until the arm arrives at the goal.
        # block_until_arm_arrives_with_prints(robot, command_client, cmd_id)
        block_until_arm_arrives(command_client, cmd_id, 5.0)
        
        time.sleep(2)
        
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.8)
        command = RobotCommandBuilder.build_synchro_command(gripper_command)
        command_client.robot_command(command)
        #Stow
        print('Carrying Finished, Stowing...')
        stow = RobotCommandBuilder.arm_stow_command()
        block_until_arm_arrives(command_client, command_client.robot_command(stow), 3.0)
        
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
        command = RobotCommandBuilder.build_synchro_command(gripper_command)
        command_client.robot_command(command)
        
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception('Threw an exception')
        return False

def bottomRow(robot, command_client):
        print("POSITION 0")
        # Example 2: Single point trajectory with maximum acceleration/velocity constraints specified such
        # that the solver has to modify the desired points to honor the constraints
        sh0 = 0.0
        sh1 = -2.0
        el0 = 2.3
        el1 = 0.0
        wr0 = -0.2
        wr1 = 0.0
        max_vel = wrappers_pb2.DoubleValue(value=1)
        max_acc = wrappers_pb2.DoubleValue(value=5)
        traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
            sh0, sh1, el0, el1, wr0, wr1, time_since_reference_secs=1.5)
        arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point],
                                                            maximum_velocity=max_vel,
                                                            maximum_acceleration=max_acc)
        # Make a RobotCommand
        command = make_robot_command(arm_joint_traj)

        # Send the request
        cmd_id = command_client.robot_command(command)
        robot.logger.info('Requesting a single point trajectory with unsatisfiable constraints.')

        # Query for feedback
        feedback_resp = command_client.robot_command_feedback(cmd_id)
        robot.logger.info('Feedback for Example 2: planner modifies trajectory')
        # time_to_goal = print_feedback(feedback_resp, robot.logger)
        # time.sleep(time_to_goal)

        # time.sleep(3)
        print('BOTTOM ROW')
        return cmd_id
        #RETURN CMD_ID
        
def middleRow(robot, command_client):
        # Example 2: Single point trajectory with maximum acceleration/velocity constraints specified such
        # that the solver has to modify the desired points to honor the constraints
        sh0 = 0.0
        sh1 = -2.0
        el0 = 1.9
        el1 = 0.0
        wr0 = 0.0
        wr1 = 0.0
        max_vel = wrappers_pb2.DoubleValue(value=1)
        max_acc = wrappers_pb2.DoubleValue(value=5)
        traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
            sh0, sh1, el0, el1, wr0, wr1, time_since_reference_secs=1.5)
        arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point],
                                                            maximum_velocity=max_vel,
                                                            maximum_acceleration=max_acc)
        # Make a RobotCommand
        command = make_robot_command(arm_joint_traj)

        # Send the request
        cmd_id = command_client.robot_command(command)
        robot.logger.info('Requesting a single point trajectory with unsatisfiable constraints.')

        # Query for feedback
        feedback_resp = command_client.robot_command_feedback(cmd_id)
        robot.logger.info('Feedback for Example 2: planner modifies trajectory')
        # time_to_goal = print_feedback(feedback_resp, robot.logger)
        # time.sleep(time_to_goal)

        time.sleep(3)
        print('MIDDLE ROW')
        return cmd_id
        
def topRow(robot,command_client):
        # Example 2: Single point trajectory with maximum acceleration/velocity constraints specified such
        # that the solver has to modify the desired points to honor the constraints
        sh0 = 0.0
        sh1 = -2.0
        el0 = 1.3
        el1 = 0.0
        wr0 = 0.6
        wr1 = 0.0
        max_vel = wrappers_pb2.DoubleValue(value=1)
        max_acc = wrappers_pb2.DoubleValue(value=5)
        traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
            sh0, sh1, el0, el1, wr0, wr1, time_since_reference_secs=1.5)
        arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point],
                                                            maximum_velocity=max_vel,
                                                            maximum_acceleration=max_acc)
        # #robot
        # robot = sdk.create_robot(options.hostname)
        # command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        # Make a RobotCommand
        command = make_robot_command(arm_joint_traj)

        # Send the request
        cmd_id = command_client.robot_command(command)
        robot.logger.info('Requesting a single point trajectory with unsatisfiable constraints.')

        # Query for feedback
        feedback_resp = command_client.robot_command_feedback(cmd_id)
        robot.logger.info('Feedback for Example 2: planner modifies trajectory')
        # time_to_goal = print_feedback(feedback_resp, robot.logger)
        # time.sleep(time_to_goal)

        # time.sleep(3)
        print('TOP ROW')
        return cmd_id
        
def stow(robot, command_client):
        # Example 3: Single point trajectory with default acceleration/velocity constraints and
        # time_since_reference_secs large enough such that the solver can plan a solution to the
        # points that also satisfies the constraints.
        sh0 = 0.0692
        sh1 = -1.882
        el0 = 1.652
        el1 = -0.0691
        wr0 = 1.622
        wr1 = 1.550
        traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
            sh0, sh1, el0, el1, wr0, wr1, time_since_reference_secs=1.5)

        arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point])

        # Make a RobotCommand
        command = make_robot_command(arm_joint_traj)

        # Send the request
        cmd_id = command_client.robot_command(command)
        robot.logger.info('Requesting a single point trajectory with satisfiable constraints.')

        # Query for feedback
        feedback_resp = command_client.robot_command_feedback(cmd_id)
        robot.logger.info('Feedback for Example 3: unmodified trajectory')
        time_to_goal = print_feedback(feedback_resp, robot.logger)
        time.sleep(time_to_goal)

        # ----- Do a two-point joint move trajectory ------

        # First stow the arm.
        # Build the stow command using RobotCommandBuilder
        stow = RobotCommandBuilder.arm_stow_command()

        # Issue the command via the RobotCommandClient
        stow_command_id = command_client.robot_command(stow)

        robot.logger.info('Stow command issued.')
        block_until_arm_arrives(command_client, stow_command_id,3)

        
        time.sleep(3)
     
        print("done")


def place_piece(robot, move, id):
    robot.time_sync.wait_for_sync()
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    joint_move_example(robot, move, id, command_client)
    
    
# Testing function
if __name__ == '__main__':
    load_dotenv()
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    
    options = parser.parse_args()
    
    sdk = bosdyn.client.create_standard_sdk('TicTacSPOT')
    robot = sdk.create_robot("192.168.80.3")
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        robot.logger.info("Powering on robot... This may take a few seconds.")
        robot.power_on(timeout_sec=40)
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot is powered on.")
        
        assert robot.has_arm(), 'Robot requires an arm to run.'
        
        blocking_stand(command_client)
        time.sleep(.35)
        place_piece(robot, (0, 1), 526)