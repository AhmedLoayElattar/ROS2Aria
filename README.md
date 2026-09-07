# ROS2Aria

**ROS 2 Jazzy driver for the Pioneer 3-DX mobile robot, developed by the E-JUST Robotics Club for RoboCup@Home Egypt 2026.**

ROS2Aria is an open-source rewrite of the Pioneer ROS driver for **ROS 2 Jazzy**. It provides the hardware interface needed to operate a rebuilt Pioneer 3-DX as part of a complete autonomous mobile-robot system, including odometry, velocity control, sonar, bumper states, battery telemetry, and gripper services.

The driver was developed as part of the team's work to turn a stock Pioneer 3-DX mobile base into a robot capable of mapping, autonomous navigation, perception, and people-finding tasks.

---

## Project at a Glance

The system combines four major layers:

```text
┌──────────────────────────────────────────────────────────┐
│                     Task / Mission Layer                 │
│          People Finding • RoboCup@Home Tasks             │
└──────────────────────────┬───────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                  Perception & Interaction                │
│       Speech Recognition • Speech Synthesis • ZED2       │
│              ROS 2 Action Server Interfaces              │
└──────────────────────────┬───────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                Mapping & Autonomous Navigation            │
│       SLAM Toolbox • Localization • Autonomous Drive      │
│                    AMCL Alternative                       │
└──────────────────────────┬───────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                       ROS2Aria                            │
│ Odometry • Velocity • Sonar • Bumpers • Battery • Gripper│
└──────────────────────────┬───────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                    Pioneer 3-DX Base                      │
│              ARIA • Motors • Encoders • Sonar             │
└──────────────────────────────────────────────────────────┘
```

---

## The Robot

A stock **Pioneer 3-DX** is essentially a mobile base. For the RoboCup@Home platform, we rebuilt the base and designed a custom sensor mast to turn it into a complete autonomous robot.

The mast carries:

- **SLAMTEC A1M8** 2D LiDAR for mapping and navigation
- **Stereolabs ZED2** stereo camera for visual perception
- **Shotgun microphone** for speech interaction

Sensor placement was designed around practical operating heights and fields of view rather than simply mounting components wherever there was space.

### Mechanical Evolution

**V1** used threaded rod and acrylic.

**V2** was redesigned around aluminum extrusion to provide a taller, more rigid, and more robust structure suitable for autonomous operation.

---

## ROS2Aria

### Why it exists

The Pioneer platform's established ROS driver existed for **ROS 1**. Our project required the robot to operate inside a modern **ROS 2 Jazzy** stack.

ROS2Aria was written to provide that hardware interface.

It handles the communication between ROS 2 and the Pioneer/ARIA base, exposing the robot's hardware capabilities to the rest of the ROS 2 system.

### Driver capabilities

ROS2Aria provides interfaces for:

- Odometry
- Velocity commands
- Sonar measurements
- Bumper states
- Battery telemetry
- Gripper services

This allows the rest of the system to interact with the Pioneer through ROS 2 interfaces instead of directly dealing with the underlying robot controller.

---

## Navigation

The navigation system was developed around **SLAM Toolbox**.

The robot was capable of:

1. Building a map of the environment
2. Localizing within the mapped environment
3. Planning autonomous motion
4. Driving through the room
5. Reaching task-relevant locations

We also tested **AMCL** as an alternative localization path using a previously saved map.

```text
                 ┌─────────────────┐
                 │   A1M8 LiDAR    │
                 └────────┬────────┘
                          │
                          ▼
                  ┌───────────────┐
                  │ SLAM Toolbox  │
                  └───────┬───────┘
                          │
                    Map / Localization
                          │
                          ▼
                  ┌───────────────┐
                  │ Autonomous    │
                  │ Navigation    │
                  └───────┬───────┘
                          │
                       cmd_vel
                          │
                          ▼
                     ROS2Aria
                          │
                          ▼
                    Pioneer 3-DX
```

### Localization

Two approaches were evaluated:

- **SLAM Toolbox** for the primary mapping/navigation workflow
- **AMCL** as an alternative localization approach against a saved map

---

## Perception & Speech

The perception and interaction stack was developed by **Omar Abdelgawad** and **Mohammed Elseiagy**.

Speech recognition and speech synthesis were integrated as **ROS 2 action servers**.

This allows the higher-level task node to treat perception and speech capabilities as standard ROS 2 goals:

```text
                Task Node
                    │
          ┌─────────┴─────────┐
          │                   │
          ▼                   ▼
 Speech Recognition     Speech Synthesis
   Action Server         Action Server
          │                   │
          └─────────┬─────────┘
                    │
                    ▼
              Task Execution
```

This architecture keeps the task logic independent from the implementation details of individual perception and interaction components.

---

## Hardware

| Component | Purpose |
|---|---|
| Pioneer 3-DX | Mobile robot base |
| ARIA | Low-level robot interface |
| SLAMTEC A1M8 | 2D LiDAR / mapping |
| Stereolabs ZED2 | Stereo vision / perception |
| Shotgun microphone | Audio input / speech interaction |
| Custom V2 mast | Sensor mounting structure |
| Aluminum extrusion | Structural frame |

---

## Software Stack

| Component | Role |
|---|---|
| ROS 2 Jazzy | Robot middleware |
| ROS2Aria | Pioneer hardware driver |
| SLAM Toolbox | Mapping and navigation |
| AMCL | Alternative localization |
| ROS 2 Actions | Speech/perception interfaces |
| ARIA | Pioneer hardware communication |

---

## Repository

This repository contains the ROS 2 driver developed for the Pioneer platform:

**ROS2Aria**

```text
https://github.com/AhmedLoayElattar/ROS2Aria
```

The driver is intended to be reusable beyond this specific RoboCup platform and is released as open source.

---

## RoboCup@Home Egypt 2026

ROS2Aria and the complete robot platform were developed for:

**RoboCup@Home Egypt 2026 — 10th Edition**

**17–18 April 2026**  
Arab Academy for Science, Technology and Maritime Transport  
New Alamein City, Egypt

The project was developed by the **E-JUST Robotics Club** team.

---

## Team & Acknowledgements

### Development

**Ahmed Loay Elattar**  
ROS2Aria / Pioneer driver

**Mohammed Abdelsabour**  
Mapping and autonomous navigation

**Omar Abdelgawad**  
Perception and speech systems

**Mohammed Elseiagy**  
Perception and speech systems

### Supervision

**Dr. Haitham El-Hussieny**

Thank you to our older team members for their guidance and support throughout the development of the platform, with special appreciation to **Mostafa Eshra**.

---

## Open Source

ROS2Aria is released as an open-source project so that others working with legacy Pioneer/ARIA platforms can use and build upon a ROS 2-compatible driver.

If you are working with a Pioneer platform and need to bring it into a modern ROS 2 system, this project may provide a useful starting point.

---

## Keywords

`ROS 2` · `ROS 2 Jazzy` · `Pioneer 3-DX` · `ARIA` · `SLAM Toolbox` · `AMCL` · `LiDAR` · `ZED2` · `RoboCup@Home` · `Mobile Robotics` · `Autonomous Navigation` · `Open Source`

---

## Citation / Reference

If this project is useful in your work, please reference the repository:

**Ahmed Loay Elattar — ROS2Aria**  
https://github.com/AhmedLoayElattar/ROS2Aria

---

## License

See the repository for the applicable license and licensing terms.
