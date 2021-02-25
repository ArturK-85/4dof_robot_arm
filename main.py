import pybullet as p
import pybullet_data
import pybullet_planning
import time
import math
import numpy as np
import logging as log

from datetime import datetime

import lib.json_file as json_file
import lib.ax_utils as ax_utils
from lib.read_write_ax import ReadWriteServoData
import lib.kinematics.kinematics as kinematics

jointsActive = [1,3,6,8]
solution_list = []

dxl_servo = ReadWriteServoData()

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.DIRECT)

box = pybullet_planning.create_box(0.05, 0.05, 0.12)
pybullet_planning.set_base_values(box, [-0.08,-0.09,0.0])
pybullet_planning.set_camera(-53.60, -20, 0.3)

obstacles = [box]

def check_available_ids():

    available_ids = json_file.load('data/arm_json/dxl_servo_list.json')

    if available_ids is False:

        available_ids = dxl_servo.scan()
        json_file.write('data/arm_json/dxl_servo_list.json', available_ids)
        print("List of servos saved to file")
    else:
        print("List of servos loaded from file")

    return available_ids


def robot_arm_init_pose():

    init_position = []

    for i in range(4):
        position = dxl_servo.read_data('PRESENT_POSITION', i+1)
        position = ax_utils.dxl_angle_to_degrees(position)
        position = math.radians(position)
        init_position.append(position)

    init_position = tuple(init_position)

    return init_position

def execute_waypoint(waypoint, available_ids):

    for i in range(len(waypoint)):

        jointPose = waypoint[i]
        print(jointPose)
        jointPose = np.rad2deg(jointPose)
        print(jointPose)

        log.basicConfig(format='%(asctime)s - %(message)s', level=log.INFO)
        log.info("Servo is waiting fo instruction")

        for i in available_ids:

            status = dxl_servo.read_data('MOVING', i)

            while True:

                if status is 0:
                    time.sleep(1/60)
                    break

                else:

                    log.basicConfig(
                       format='%(asctime)s - %(message)s',
                       level=log.WARNING
                    )
                    log.warning("Servo is busy!")
                    status = dxl_servo.read_data('MOVING', i)
                    continue

            angles = (jointPose[i-1])
            angles = ax_utils.degrees_to_dxl_angle(angles)
            print(angles)

            dxl_servo.write_data('GOAL_POSITION', int(angles), i)
            time.sleep(1/60)

        log.basicConfig(format='%(asctime)s - %(message)s', level=log.INFO)
        log.info("Execute!")

        time.sleep(1)

def main():

    available_ids = check_available_ids()

    for i in available_ids:
        dxl_servo.write_data('MOVING_SPEED', 150, i)

    robot = kinematics.LoadURDF("data/arm_desc/robot.urdf").get_desc()

    initPose = robot_arm_init_pose()

    startPose = kinematics.InvKinematics(robot).calculate_kinematics(
       [-0.0, -0.02, 0.02]
    )

    endPose = kinematics.InvKinematics(robot).calculate_kinematics(
       [0.15, -0.22, 0.05]
     )

    position_list = np.array(
       [[startPose, endPose],
        [endPose, startPose]]
    )

    for startPose, endPose in position_list:

        solution = kinematics.MotionPlanning(robot, jointsActive).solution(
           startPose,
           endPose,
           obstacles=obstacles
        )

        solution_list.append(solution[0])

    for i in range(len(position_list)):
        del solution_list[i][0]

    for i in range(len(solution_list)):
        waypoint = kinematics.MotionPlanning(robot, jointsActive).get_waypoints(
           solution_list[i]
        )

        execute_waypoint(waypoint, available_ids)

    dxl_servo.close()

if __name__ == "__main__":
    main()
