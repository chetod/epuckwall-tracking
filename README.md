This controller is designed to solve a maze by combining basic robotic behaviors using a state machine. It uses the e-puck robot equipped with distance sensors, wheel encoders, and motors. The robot performs wall-following, turns, and curved maneuvers to navigate through the maze.

Key Components
-State Machine:
1.follow_wall: Default behavior to follow the wall on the left side.
2.turn_-90: Turns left in place when an obstacle is detected ahead.
3.curve_+90: Makes a smooth right turn when the wall ends.

-Sensors Used:
ps5, ps6: Detect the left and front-left distances.
ps0, ps7: Detect obstacles in front.

Encoders: Measure wheel rotation to calculate speed.

-Control Logic:
Linear and angular velocities are computed based on sensor input.

A proportional controller (gains kd, kd2) adjusts the robot’s heading to follow the wall.

Speeds are converted to wheel commands using a kinematic model.

Functions
get_wheels_speed(...): Computes wheel angular speeds from encoder readings.

get_robot_speeds(...): Converts wheel speeds to linear and angular robot speeds.

wheel_speed_commands(...): Converts desired linear/angular speeds to individual wheel speeds.

follow_wall_to_left(...): Uses a control law to maintain a safe and consistent distance from the left wall.

Behavior Transition Conditions
From follow_wall → curve_+90: Wall disappears on the left.

From follow_wall → turn_-90: Obstacle detected in front.

From turning states → follow_wall: After a fixed number of steps (via counter).

Debugging Output
The current state and selected proximity sensor readings are printed each loop cycle to assist with debugging.

