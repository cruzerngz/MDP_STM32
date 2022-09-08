# MDP_STM32
STM32 controller code for CE3004

# STM Cube IDE

## Setup

## Adding source folders to project
The IDE is very finnicky about this.
Here are the steps to correctly add a source directory
(containing source and header files)
into your project.




# UART API
Rough documentation of the UART-robot interface

UART communication hard rules:
- Only one char (command) sent at a time
- Command has to be a valid ASCII character
- Each sent char will be acknowledged with one of 3 replies
- Incorrect commands / improperly received commands will have to be sent again

UART general communication/acknowledgement loop
- Instruction sent from Pi to MCU
- MCU sends partial ACK `char "."`
- Further instruction (if applicable) sent from Pi to MCU
- MCU sends full ACK `char ";"` on complete instruction
- MCU replies with error `char "?"` if an incorrect command is sent. Fallback to previous state.

UART "state machine" replies
| UART | Description |
| --- | --- |
| `.` | Partial ack, continue with next command |
| `;` | Full ack |
| `?` | Unknown command |

---

## Startup
On power on, the robot enters an idle state and waits for a UART signal to set the drive mode.

| Drive Mode | UART | Description |
| --- | --- | --- |
| Fine control mode | `f` | Calibrated movement, used in path following. |
| Toy car mode | `t` | Movement using WASD keys, for debugging purposes. |
| Mode select | `\` | Return to mode selection |
<!-- | Slave mode | `s` | Movement set using WASDX, todo!() | -->

---

## Fine control mode
TODO!()


---

## Toy car mode
The toy car mode is primarily used to determine the control parameters needed for fine control mode.
Pressing the WASD keys will result in the car moving/turning its wheels.

### Movement

| UART | Description |
| --- | --- |
| `w` | Forward |
| `a` | Backward |
| `s` | Wheels left |
| `d` | Wheels right |
| `x` | Center wheels |
| `z` | Stop motors |

### Modifiers
0-indexed modifiers

| UART | Description |
| --- | --- |
| `m` | Modify motor speed |
| `n` | Modify servo magnitude |

<!-- ### Motor and Servo parameters

Motor power and steering angle are modified by:
 - Sending a command listed below
 - Sending a number within the accepted ranges.

sending `p 1` will set the motor power level to its lowest setting.

sending `l 4` will set the steering angle to its largest setting.

| UART | Description |
| --- | --- |
| `p` | Motor **p**ower level (1-8) |
| `l` | Steering **l**ock level (1-4) | -->


---
