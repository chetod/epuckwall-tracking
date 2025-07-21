"""maze solver controller"""

# This file contains only a bare-bones structure with initialization of variables,
# sensor reading, a few functions and a simple state machine to solve a maze. 
# You can use it as a template for your solution.

# Author: Felipe N. Martins
# 05-MAR-2024

from controller import Robot, DistanceSensor, Motor, Compass
import numpy as np

#-------------------------------------------------------
# Initialize variables

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]
delta_t = robot.getBasicTimeStep()/1000.0    # [s]

# states for the state machine: modify according to your state machine
states = ['follow_wall', 'turn_-90', 'curve_+90']
current_state = states[0]

# counter: used to maintain an active state for a number of cycles
counter = 0
COUNTER_MAX = 100

# Robot wheel speeds: initial values are zero
wl = 0.0    # angular speed of the left wheel [rad/s]
wr = 0.0    # angular speed of the right wheel [rad/s]

# Robot linear and angular speeds: initial values are zero
u = 0.0    # linear speed [m/s]
w = 0.0    # angular speed [rad/s]

# e-puck Physical parameters for the kinematics model (constants)
R = 0.0205    # radius of the wheels: 20.5mm [m]
D = 0.0520    # distance between the wheels: 52mm [m]

# Controller gains for wall-following - define how the reaction of the robot will be:
# higher controller gain will result in faster reaction, but it can cause oscillations
kd = 0.003
kd2 = 0.003
d_desired = 200 # in units of the proximity sensor (higher is closer)

#-------------------------------------------------------
# Initialize devices

# proximity sensors: measure distance to walls and obstacles
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# encoders: measure the angular position of the wheels in radians
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)
    
oldEncoderValues = []

# motors    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

############################################################################
# Functions
# Complete the functions below and create new ones to complete the mission

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    """Computes speed of the wheels based on encoder readings"""
    # This function is already working.
    wl = (encoderValues[0] - oldEncoderValues[0])/delta_t
    wr = (encoderValues[1] - oldEncoderValues[1])/delta_t

    return wl, wr


def get_robot_speeds(wl, wr, r, d):
    """Computes robot linear and angular speeds"""

    # Fill in the proper equations:
    u = R * (wr + wl) / 2.0
    w = R * (wr - wl) / D


    return u, w


def wheel_speed_commands(u_d, w_d, D, R):
    """Converts desired speeds to wheel speed commands
    Inputs:
        u_d = desired linear speed for the robot [m/s]
        w_d = desired angular speed for the robot [rad/s]
        R = radius of the robot wheel [m]
        D = distance between the left and right wheels [m]
    Returns:
        wr_d = desired speed for the right wheel [rad/s]
        wl_d = desired speed for the left wheel [rad/s]
    """

    # Fill in the proper equations:
    wl_d = (u_d - (w_d * D / 2.0)) / R
    wr_d = (u_d + (w_d * D / 2.0)) / R


    return wl_d, wr_d


def follow_wall_to_left(kd, kd2, d_l, d_fl, d_desired):
    """ Follows the wall to the left of the robot.
    Input Parameters: 
        kd = controller gain for controlling the distance to the wall;
        kd2 = controller gain for keeping the robot parallel to the wall;
        d_desired = desired robot distance to the wall in sensor units;
        d_l = "distance" to the left wall measured by ps5;
        d_fl = diagonal "distance" to the front-left wall measured by ps6;
        !! sensor values are inversely proportional to the distance !!
        Returns:
        u_ref = reference linear speed command;
        w_ref = reference angluar speed command.
    """
    
    # Fill in the proper equations or values:
    U_MAX = 0.06  # reasonable forward speed
    u_ref = U_MAX

# control law: deviation from desired distance
    e_d = d_l - d_desired
    e_theta = d_fl - d_l

    w_ref = -kd * e_d - kd2 * e_theta

    
    return u_ref, w_ref

############################################################################


#---------------------------------------------------------------------------
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Implement the see-think-act cycle:

    # ------------------ See ---------------------
    # Sensor values are being placed in the corresponding lists.
    # This part is already working.
    # proximity sensors:
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    # encoders:
    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    # [rad]
    # Update old encoder values if not done before
    if len(oldEncoderValues) < 2:
        for i in range(2):
            oldEncoderValues.append(encoder[i].getValue())   

    # ------------------ Think ---------------------
    # Implement the state machine to select proper behaviors
    # The example below is a simple maze-solver state machine. You can use it
    # as a stating point.
    
    # Actions:
    if current_state == 'follow_wall':
        u_d, w_d = follow_wall_to_left(kd, kd2, psValues[5], psValues[6], d_desired)

    if current_state == 'turn_-90':
        u_d = 0.0
        w_d = -0.5

    if current_state == 'curve_+90':
        u_d = 0.03
        w_d = 0.6

    # Transitions:
    if current_state == 'follow_wall':
        if psValues[5] < 80 and psValues[6] <80:
            current_state = 'curve_+90'
            counter = 20    # higher initial value for a shorter curve
        if psValues[0] > 150 or psValues[7] > 170:
            current_state = 'turn_-90'
            counter = 0
    
    if current_state == 'curve_+90':
        if counter >= COUNTER_MAX:
            current_state = 'follow_wall'

    if current_state == 'turn_-90':
        if counter >= COUNTER_MAX:
            current_state = 'follow_wall'

    # increment counter
    counter += 1

    # update old encoder values for the next cycle
    oldEncoderValues = encoderValues
    
    # ------------------ Act ---------------------
    # Set motor speeds with the values defined by the state machine.
    # This part is already working. You only need to complete the function.
    leftSpeed, rightSpeed = wheel_speed_commands(u_d, w_d, D, R)
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)


    # To help on debugging:        
    print(f'Current state = {current_state}, ps = {psValues[5]:.2f}, {psValues[5]:.2f}, {psValues[0]:.2f}, {psValues[7]:.2f}')

    # End of the loop. Repeat all steps while the simulation is running.