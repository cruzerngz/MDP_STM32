# MDP_STM32
STM32 controller code for CE3004

# STM Cube IDE

## Setup

## Adding source folders to project
The IDE is very finnicky about this.
Here are the steps to correctly add a source directory
(containing source and header files)
into your project.

### Adding directories to project
- Go to File -> Import -> File System
- Fill in the path to directory
- Check the box next to directory
- Fill in the "into folder" field
- Click on "Finish"
- Check that the directory is imported into the correct loc in your workspace
- Add the folder's Inc directory to [your compiler's include path](#adding-include-paths)

### Adding files into project
**For new files**
- Create a file in the STM IDE
- That's it

**For external files**
- Create a file in the STM IDE
- Paste contents of external file


### Adding include paths
- Click on your STM project in the project explorer
- Right click on the project root
- Go into properties -> C/C++ General -> Paths and Syms
- Change the configuration to **All configurations** \
(you might want to compile in release mode after completing debugging)
- Add the include path under "GNU C"


---

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
| `_` | Accepting numeric input |
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
This mode is used to control the movement of the car as accurately as possible. There are 2 types of commands available.

| Command | Description |
| --- | --- |
| Variable | First char sent sets the move mode. Next sequence of chars sent (most significant digit -> least significant digit) sets the magnitude. Finally, `;` signals the MCU to run the instruction. |
| Predefined | Predefined set of moves for the car to take |


### Variable movements (incomplete)
| UART sequence | Description |
| --- | --- |
| `m-[number sequence]-;` | Move the car forward by X centimeters |
| `t-[number sequence]-;` | Turn the car clockwise by X degrees |
<!-- | `m-< f / b >-[number sequence]-;` | Move the car forwards/backwards by X centimeters | -->
<!-- | `t-< r / l >-[number sequence]-;` | Turn the car clockwise/anticlockwise by X degrees | -->
<!-- | `i-f-[number sequence]-;` | In-place rotate by by X degrees (forward-backward) |
| `i-b-[number sequence]-;` | In-place rotate by by X degrees (backward-forward) | -->


### Predefined movements (also incomplete)
| UART sequence | Description |
| --- | --- |


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
