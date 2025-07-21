# Maze Solver Controller for e-puck Robot

This project implements a basic **maze-solving behavior** for the **e-puck robot** in the **Webots simulator**. The robot uses a **finite state machine** and a **wall-following algorithm** to navigate through a maze using its distance sensors.

## 👨‍💻 Author

- Felipe N. Martins  
- Date: 05-MAR-2024  

---

## 🧠 Behavior Overview

The robot operates based on 3 main states:

1. **`follow_wall`** – Follows the wall on the left side using proximity sensors.
2. **`turn_-90`** – Performs an in-place 90° left turn when an obstacle is ahead.
3. **`curve_+90`** – Makes a smooth right curve when the left wall disappears.

State transitions are controlled using sensor values and a step counter.

---

## 🧩 Sensor & Actuator Setup

- **Proximity Sensors (ps0–ps7):**
  - `ps5`: Left wall sensor
  - `ps6`: Front-left diagonal sensor
  - `ps0`, `ps7`: Front sensors to detect obstacles

- **Encoders:**
  - Left and right wheel encoders for calculating wheel speeds.

- **Motors:**
  - Left and right wheel motors with velocity control.

---

## ⚙️ Key Parameters

- `R = 0.0205`: Wheel radius (meters)
- `D = 0.0520`: Distance between wheels (meters)
- `kd`, `kd2`: Controller gains for wall-following
- `d_desired = 200`: Desired sensor value for distance to wall

---

## 🧮 Functions

- `get_wheels_speed(...)` – Calculates wheel angular speed from encoders
- `get_robot_speeds(...)` – Computes linear and angular speed of robot
- `wheel_speed_commands(...)` – Converts desired speeds to motor commands
- `follow_wall_to_left(...)` – Wall-following control law using proximity sensors

---

## 🔁 State Transitions

| Current State   | Condition                                  | Next State     |
|-----------------|--------------------------------------------|----------------|
| `follow_wall`   | Wall disappears (ps5, ps6 low)             | `curve_+90`    |
| `follow_wall`   | Obstacle ahead (ps0 or ps7 high)           | `turn_-90`     |
| `turn_-90`      | Counter reaches limit                      | `follow_wall`  |
| `curve_+90`     | Counter reaches limit                      | `follow_wall`  |

---

## 🐞 Debugging Output

The script prints useful information each timestep:

```plaintext
Current state = follow_wall, ps = 310.00, 310.00, 40.00, 60.00
