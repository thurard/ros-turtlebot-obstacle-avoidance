# 🐢 ROS TurtleBot — FSM Obstacle Avoidance

Autonomous obstacle-avoidance behaviour for a **TurtleBot 2**, written in **ROS (rospy)**
and structured as a **finite-state machine**. The robot can be driven manually with a
joystick, switch to an autonomous cruise mode, and run a reflex avoidance sequence when
an obstacle is detected by the **LIDAR** or the **bumpers**.

[![Watch the demo](https://img.youtube.com/vi/6W-9P3nt3xU/hqdefault.jpg)](https://youtu.be/6W-9P3nt3xU)

> ▶️ **[Watch the demo on YouTube](https://youtu.be/6W-9P3nt3xU)** — Gazebo simulation,
> then real-robot avoidance triggered by the bumper and by the LIDAR.

---

## Contents

- [Overview](#overview)
- [How it works — the state machine](#how-it-works--the-state-machine)
- [Perception & motion](#perception--motion)
- [Repository structure](#repository-structure)
- [Running it](#running-it)
- [Report](#-report)
- [Authors](#authors)

---

## Overview

Ground-robotics project (SYSMER track, SeaTech — University of Toulon, 2025). The goal:
give a TurtleBot the ability to **navigate autonomously and avoid obstacles** in an
unknown environment, while keeping a manual override.

The robot's intelligence is organised as a **finite-state machine (FSM)**: at any instant
it is in exactly one state that defines a precise action (drive, reverse, turn, wait), and
transitions are driven by sensor data and user input.

---

## How it works — the state machine

```
START ──► JoyControl ◄──────────────► AutonomousMode1 ──(obstacle)──► Stop1
             (manual)   button / joystick     (cruise)                   │
                                                  ▲                       ▼
                                                  │                     Recule
                                                  │                       │
                                              Stop3 ◄── Rotate ◄── Stop2 ◄─┘
```

| State | Behaviour |
|---|---|
| **JoyControl** | Manual joystick teleop, with velocity smoothing. Press **button A** → autonomous. |
| **AutonomousMode1** | Cruise: drive straight at `vmax/3`, while watching the LIDAR and bumpers. Moving the joystick returns to manual. |
| **Stop1** | Emergency stop the instant an obstacle is detected. |
| **Recule** | Reverse (`-vmax/3`) to back away from the obstacle. |
| **Stop2** | Brief stop to kill inertia before turning. |
| **Rotate** | Turn on the spot (`wmax/2`) to change heading. |
| **Stop3** | Final pause, then hand control back to `AutonomousMode1`. |

The avoidance states chain automatically via **cycle counters**: the control loop runs at
**10 Hz**, and each step lasts ~10 cycles (≈ 1 s) before moving on. This makes the robot
**stop and fully re-orient** before resuming, rather than trying to swerve around the
obstacle.

---

## Perception & motion

- **LIDAR** (`sensor_msgs/LaserScan`, callback `processScan`): scans all beams; if any
  valid range is below **0.5 m**, an obstacle flag is raised.
- **Bumpers** (`kobuki_msgs/BumperEvent`, callback `processBump`): a physical contact
  also raises the obstacle flag — a last-resort safety layer behind the LIDAR.
- **Velocity smoothing** (`smooth_velocity`): a ramping algorithm limits acceleration so
  the wheels don't slip and the odometry stays clean, instead of applying raw setpoints.

---

## Repository structure

```
.
├── turtlebot_fsm.py     % Main ROS node: RobotBehavior class + FSM
├── fsm.py               % Finite-state-machine engine (from the course)
├── Rapport.pdf          % Full project report (French)
└── README.md
```

> **Note:** the node imports the FSM engine with `from fsm import fsm`, so `fsm.py`
> (provided with the course) must sit next to it. If you cannot redistribute it, keep the
> import and mention it as an external dependency.

---

## Running it

**Requirements:** ROS (Melodic/Noetic), `rospy`, the TurtleBot 2 / Kobuki stack, and a
joystick.

```bash
roscore
# bring up the TurtleBot (or its Gazebo simulation) and the joystick node, then:
rosrun <your_package> turtlebot_fsm.py
```

The node subscribes to `joy`, `/scan` (LIDAR) and `/mobile_base/events/bumper`, and
publishes velocity commands on `/mobile_base/commands/velocity` at 10 Hz.

---

## 📄 Report

Full project report — FSM design, LIDAR processing, avoidance strategy and possible
extensions (odometry-based turns, SLAM) — in French:

**➡️ [Rapport.pdf](Rapport.pdf)**

---

## Authors

**Tom Hurard**, **Antoine Hureau** & **Amélie Duchemin** — engineering students, SeaTech
(University of Toulon), SYSMER, 2025. Supervised by Vincent Hugel.
