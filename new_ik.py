import pybullet as p
import time
import math
import numpy as np
import logging as log

from datetime import datetime

import pybullet_data
import pybullet_planning

import ax_utils
from read_write_ax import ReadWriteServoData

dxl_servo = ReadWriteServoData()

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.DIRECT)

p.resetSimulation()

robot = p.loadURDF("robot.urdf", [0,0,0])
joints = p.getNumJoints(robot)
#print(p.getJointInfo(robot, 8))

jointsActive = [1,3,6,8]

pos = [-0.0, -0.02, 0.02]
jointAngle = []

box = pybullet_planning.create_box(0.05, 0.05, 0.12)
pybullet_planning.set_base_values(box, [-0.08,-0.09,0.0])

obstacles = [box]

pybullet_planning.set_camera(-53.60, -20, 0.3)

jointPoses = p.calculateInverseKinematics(robot, 11, pos)
endPoses = p.calculateInverseKinematics(robot, 11, [0.15, -0.22, 0.05])

limits=pybullet_planning.get_custom_limits(robot, jointsActive, circular_limits=pybullet_planning.CIRCULAR_LIMITS)
print(limits)
joints = pybullet_planning.get_joints(robot)
print(joints)
#start_conf = pybullet_planning.get_joint_positions(robot, jointsActive)
#print(start_conf)
sample_fn = pybullet_planning.get_sample_fn(robot, jointsActive, custom_limits=limits)
print(sample_fn)
extend_fn = pybullet_planning.get_extend_fn(robot, jointsActive, resolutions=None)
print(extend_fn)
collision_fn = pybullet_planning.get_collision_fn(robot, jointsActive, obstacles=obstacles, custom_limits=limits)
print(collision_fn)

solution = pybullet_planning.lazy_prm(
   start_conf=jointPoses,
   end_conf=endPoses,
   sample_fn=sample_fn,
   extend_fn=extend_fn,
   collision_fn=collision_fn)

solution2 = pybullet_planning.lazy_prm(
   start_conf=endPoses,
   end_conf=jointPoses,
   sample_fn=sample_fn,
   extend_fn=extend_fn,
   collision_fn=collision_fn)

waypoint = pybullet_planning.waypoints_from_path(solution[0])

waypoint_return = pybullet_planning.waypoints_from_path(solution2[0])

for i in range(4):
    dxl_servo.write_data('MOVING_SPEED', 150, i+1)



#print(p.getJointStates(robot,[1,3,6,8])

#while  True:

#    p.setJointMotorControlArray(robot, jointsActive, p.POSITION_CONTROL, targetPositions=jointPoses)
#    p.stepSimulation()

#    for i in range(len(solution[0])):
#       p.setJointMotorControlArray(robot, jointsActive, p.POSITION_CONTROL, targetPositions=solution[0][i])
#       p.stepSimulation()
#       time.sleep(0.1)

def execute_waypoint(waypoint):

    for i in range(len(waypoint)):

        jointPose = waypoint[i]
        print(jointPose)
        jointPose = np.rad2deg(jointPose)
        print(jointPose)

        log.basicConfig(format='%(asctime)s - %(message)s', level=log.INFO)
        log.info("Servo is waiting fo instruction")

        for i in range(4):

            status = dxl_servo.read_data('MOVING', i+1)

            while True:

                if status is 0:
                    time.sleep(1/60)
                    break

                else:

                    log.basicConfig(format='%(asctime)s - %(message)s', level=log.WARNING)
                    log.warning("Servo is busy!")
                    status = dxl_servo.read_data('MOVING', i+1)
                    continue

            angles = (jointPose[i])
            angles = ax_utils.degrees_to_dxl_angle(angles)
            print(angles)

            dxl_servo.write_data('GOAL_POSITION', int(angles), i+1)
            time.sleep(1/60)

        log.basicConfig(format='%(asctime)s - %(message)s', level=log.INFO)
        log.info("Execute!")

        time.sleep(1)

execute_waypoint(waypoint=waypoint)
execute_waypoint(waypoint=waypoint_return)

dxl_servo.close()
