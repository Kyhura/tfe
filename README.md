# Biologically-inspired architecture for a humanoid robot: from low-level hardware to elementary control

Master thesis under the supervision (promotor): Professor Bernard Boigelot.

## Overview

In the RoboCup competition, humanoid robots play soccer matches and the Montefiore Team aims to participate in the KidSize category. Achieving this challenge requires a robust and fully integrated platform capable of human-inspired locomotion.

This thesis focuses on the development of every layer of the robot following a bottom-up approach. At the hardware-level, we completed two custom boards: the RM-28 servomotor board integrating motor control, various sensors and a CAN channel, and the multifunction board distributing the power throughout the robot and orchestrating a periodic CAN cycle for the communication with the servomotors and a USB connection with the motherboard. 

The software embedded in the servomotors includes three elementary PID controllers to drive the motor in position, velocity, and torque which were validated in a real environment demonstrating good reference tracking. 

The logic of the motherboard is organised into software units that handle state management, data processing, and reflex execution. A bio-inspired locomotion system is developed with control units, similar to human neurones, which are stimulated and inhibited to produce human-like motion. 

## Structure

- **common**: Contains shared header files defining the application message protocol, the information IDs, and constants used across the robot.

- **stm32**: Contains the embedded software running on the STM32 microcontrollers of the MF board and servomotors.
  - Note that the code of the MF board is not present in the repository as my contribution to this component was focus on bug identification and suggesting improvements rather than developing new functionalities.
  - **controllers**: Manage the PID controllers logic and the motor's control
  - **sensors**: Manage the setup and the operations of the RM-28's sensors
  - **utils**: Utility functions used by the RM-28 software
    
- **x86-64**: Contains the high-level software of the motherboard.
  - **expert**: Software units responsible for updating a specific piece the world knowledge based on information received from the daughter boards as well as from other tasks thanks to the supervisor.
  - **reflex**: Software units reproducing human behaviours to achieve human-inspired locomotion by executing algorithms to determine the state parameters of the actuators (PID gains, reference, ...).
  - **supervisor**: Software units orchestrating the USB communication channel by receiving and handling the uplink buffer from the MF board and distributing the incoming messages to each interested task. It also initiates the other units, starts the ongoing dashboard and transmits the robot's state to it in real-time, and manages the downlink buffer transmitted to the MF board.
  - **utils**: Utility functions used by the motherboard software
  - **world**: Software units containing the robot's database, representing all the knowledge of the MB about the state of the environment and of the robot, from its low-level actuators, such as the angular position of a servomotor, to the high-level logic, such as the next action to perform to maintain a walking path.

## Usage

This repository only acts a representation of the contribution of this thesis. Therefore, it does not contain all the required files to run the project: it cannot operate on its own.
