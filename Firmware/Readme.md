PROJECT NAME
------------
Maze Solver Robot – Firmware


OVERVIEW
--------
This firmware controls an autonomous maze-solving robot using
distance sensors, wheel encoders, and PID-based motor control.
The robot navigates a maze by detecting walls and making
real-time movement decisions.

The entire firmware is implemented in a single Arduino .ino
file for simplicity, fast debugging, and competition use.


PLATFORM
--------
Development Environment : Arduino IDE
Programming Language    : Embedded C/C++
Communication Interfaces:
- I2C (VL53L1X distance sensors)
- GPIO and PWM (motor control)
- External interrupts (sensor data ready signals)


HARDWARE INTERFACES
-------------------

Distance Sensors:
- Sensor Type : VL53L1X Time-of-Flight
- Front Sensor I2C Address : 0x2A
- Right Sensor I2C Address : 0x2B
- Left Sensor I2C Address  : 0x2C
- XSHUT pins are used during startup to assign unique I2C addresses
- Sensors operate in continuous ranging mode

Encoders:
- Quadrature encoders on left and right motors
- Used for distance measurement and rotation accuracy

Motor Driver:
- Dual DC motor driver
- Direction control using IN1 / IN2 pins
- Speed control using PWM signals


CONTROL STRATEGY
----------------

Forward Motion:
- Robot moves forward using PID-based wall-following control
- Encoder feedback ensures accurate travel distance

PID Wall Following:
- Error calculated using left and right wall distances
- Proportional and Derivative terms are used
- Integral term is limited to avoid windup
- Motor speeds are dynamically corrected to maintain alignment

Emergency Stop:
- Front distance sensor continuously monitored
- If obstacle distance drops below safe threshold,
  motors are stopped immediately


INTERRUPT HANDLING
------------------
- Hardware interrupts are used for all distance sensors
- Interrupt Service Routines (ISR) set ready flags
- Sensor data is safely read in the main loop
- This improves responsiveness and avoids blocking delays


NAVIGATION LOGIC
----------------
The maze-solving logic follows these rules:

1. If the front path is clear, move forward
2. If front and both sides are blocked, rotate
3. If front is blocked and one side is open, turn toward the open side
4. After turning, move forward to the next cell

This enables autonomous traversal of structured maze layouts.


ROTATION CONTROL
----------------
- Rotations are performed using encoder feedback
- Acceleration and deceleration ramps are applied
- Reduces overshoot and improves turning accuracy


STATUS INDICATION
-----------------
- An onboard LED is used for system status
- LED ON  : Sensor initialization error
- LED OFF : Normal operation


FILE STRUCTURE
--------------
Firmware/
└── MazeSolver.ino

All firmware logic is contained within a single file.


DESIGN DECISIONS
----------------
- Single-file firmware chosen for:
  - Faster debugging
  - Reduced complexity
  - Competition-oriented development
- Interrupt-based sensing improves real-time behavior
- Encoder-based motion improves movement precision


LIMITATIONS
-----------
- No maze memory or map storage
- No shortest-path optimization (e.g., flood fill)
- Tuned for structured maze environments


APPLICATIONS
------------
- Maze-solving robotics competitions
- Embedded systems and control learning
- Autonomous navigation experiments


AUTHOR
------
<Your Name>
Maze Solver Robot – Firmware
