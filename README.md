🧭 Maze Solver Robot
📌 Overview

This project presents an autonomous maze-solving robot designed using a custom PCB and Arduino-based firmware.
The robot is capable of navigating a maze using wall detection, motor control, and decision-making logic, making it suitable for robotics competitions, embedded systems learning, and experimentation.

The focus of this project is on:

Compact PCB design

Reliable motor and sensor interfacing

Real-time control logic

Practical implementation under competition constraints

🧠 System Description

The maze solver robot operates by sensing walls using distance sensors and controlling two DC motors through a motor driver.
Based on sensor input, the robot makes movement decisions at junctions and follows the maze until it reaches the goal.

Key characteristics:

Autonomous operation

Sensor-based navigation

Designed for structured maze environments

🧩 Hardware
🔧 PCB Design

Custom 2-layer PCB

Designed for maze-solving robot applications

Integrates:

Microcontroller

Motor driver interface

Sensor connectors

Buck converter for power regulation

Push buttons for user input

Status LEDs for debugging

Only Gerber fabrication files are provided for the hardware.

📁 Hardware files:

Hardware/
└── Gerber/
    ├── MazeSolver_Gerber_v1.0.zip
    └── Gerber_README.txt

⚙️ Firmware
🧾 Platform

Arduino IDE

Single-file firmware implementation (.ino)

🧠 Code Structure

The entire firmware is written in one Arduino .ino file and is logically divided using comments into:

Pin definitions

Sensor reading logic

Motor control

PID control

Maze-solving logic

Main loop

This structure was chosen to:

Simplify debugging

Enable rapid testing during competitions

Reduce integration complexity

📁 Firmware files:

Firmware/
└── MazeSolver.ino

🧭 Navigation & Control Logic

Wall detection using distance sensors

Motor control using PWM signals

PID-based correction for straight movement

Decision-making at junctions

Cell-based or wall-following maze-solving strategy

🔋 Power System

External battery input

On-board buck converter for regulated logic voltage

Separate routing for motor and logic supplies

🏁 Applications

Maze-solving robotics competitions

Embedded systems and PCB design practice

Autonomous navigation experiments

Academic projects and demonstrations

⚠️ Notes & Limitations

Only Gerber files are included; schematic source files are not provided

Firmware is implemented as a single file

Designed for educational and competition use

<img width="800" height="533" alt="image" src="https://github.com/user-attachments/assets/8e95f7a8-8e93-48a8-a345-4b369e716678" />

<img width="800" height="906" alt="image" src="https://github.com/user-attachments/assets/c51cddb7-0671-414d-8ba3-addee3f850fb" />


