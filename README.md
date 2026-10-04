# Minibot: Indoor Autonomous Navigation Robot

## About the Robot

Minibot is an indoor autonomous navigation robot built around a differential-drive platform with two powered wheels and two caster wheels.

The robot uses a **ZED 2i stereo camera** for perception and an **NVIDIA Jetson AGX Orin** for onboard computation. Two wheel encoders are connected through a single **ESP32** microcontroller for obtaining wheel feedback.

The ZED 2i is mounted on an adjustable-height camera mount, allowing its position to be modified as required.

### Hardware

- NVIDIA Jetson AGX Orin
- ZED 2i Stereo Camera
- Two differential-drive motors
- Two caster wheels
- Two wheel encoders
- ESP32 microcontroller
- Adjustable-height camera mount

---

## Development

The project follows a simulation-first approach.

### Simulation

Before building the physical robot, I developed a complete simulation of Minibot in **Gazebo Ignition**.

The simulated robot is capable of autonomous navigation using:

- **ROS 2 Nav2** for navigation
- **RTAB-Map** for SLAM

The simulation was used to develop and test the navigation pipeline before deploying it on the physical robot.

### Physical Robot

After completing the simulation, I built the physical Minibot.

The robot is now mechanically complete and drives successfully, with the motor controller fully functional.

The next step is to deploy and test the complete **SLAM and autonomous navigation pipeline on the real robot**.

---

## Current Status

| Component | Status |
|---|---|
| Gazebo Ignition simulation | Complete |
| Nav2 navigation in simulation | Complete |
| RTAB-Map SLAM in simulation | Complete |
| Physical robot assembly | Complete |
| Motor controller | Working |
| Wheel encoder integration | Implemented |
| Jetson AGX Orin integration | Complete |
| ZED 2i integration | Complete |
| SLAM on physical robot | In progress |
| Autonomous navigation on physical robot | Upcoming |

---

## Documentation

Detailed documentation for the project is currently being prepared.

It will cover the complete development process, including the simulation, mechanical construction, electronics, software, and deployment on the physical robot.

I am also preparing videos demonstrating the different stages of the project. The documentation and videos will be added to this repository as they are completed.
