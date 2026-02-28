# ESP32 Closed-Loop DC Motor Control & Telemetry

This project implements a robust closed-loop Proportional-Integral (PI) control system for a Direct Current (DC) motor using an ESP32 microcontroller. It features a custom C++ PID library with anti-windup protection, hardware-interrupt-based encoder reading, and a real-time Python/Streamlit dashboard for telemetry visualization.

## Features
* **Closed-Loop Speed Control**: Maintains target RPM against physical load variations using a finely tuned PI controller.
* **Custom PID Library**: Lightweight, Object-Oriented C++ library (`PID.h` and `PID.cpp`) implementing the discrete-time difference equation with an integral clamping mechanism to prevent windup.
* **Hardware Interrupts**: Accurate, non-blocking quadrature encoder pulse counting using ESP32 `IRAM_ATTR` interrupts.
* **Real-Time Telemetry**: Asynchronous JSON data transmission over UART at 10Hz.
* **Streamlit Dashboard**: A Python-based GUI to visualize target vs. actual speed, PWM output, and system status in real-time.
* **Hardware Override**: A physical push-button to safely bypass the PID controller and dynamically brake the motor.

## Hardware Architecture
### Components Needed
* ESP32 Development Board
* DC Motor with integrated Quadrature Encoder
* L298N H-Bridge Motor Driver
* 10kΩ Potentiometer (for RPM set-point)
* Push Button (for hardware stop)
* 12V DC Power Supply (for L298N and Motor)

### Pin Mapping
| Component | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **Encoder Phase A** | `GPIO 35` | Configured as `INPUT_PULLUP` with `FALLING` edge interrupt |
| **Encoder Phase B** | `GPIO 34` | Configured as `INPUT_PULLUP` |
| **Potentiometer** | `GPIO 32` | Analog input for speed reference |
| **Stop Button** | `GPIO 33` | Configured as `INPUT_PULLUP` |
| **L298N IN1** | `GPIO 25` | Direction Control A |
| **L298N IN2** | `GPIO 27` | Direction Control B |
| **L298N PWM** | `GPIO 26` | Speed Control (10-bit PWM resolution, 5000 Hz) |
| **Status LED** | `GPIO 2` | Onboard LED to indicate loop execution |

## Software Implementation
### 1. ESP32 Firmware (`motor_control.ino`)
The firmware runs on a deterministic 15ms sampling loop. It calculates the current RPM using the following logic:
`currentRPM = (pulseCount * 60.0) / (115 * dt)`
*Note: The constant `115` accounts for the encoder's pulses per revolution multiplied by the gearbox reduction ratio.*

The PI controller is tuned using the Ziegler-Nichols method, heavily dampened for smooth velocity control without high-frequency noise amplification:
* **Kp**: 1.5
* **Ki**: 60.0
* **Kd**: 0.0

### 2. Streamlit Dashboard (`dashboard.py`)
The Python dashboard reads the JSON telemetry payload from the serial port. 

**Telemetry JSON Format:**
```json
{"target": 85.5, "real": 84.2, "pwm": 450, "stop": 0}
```

# Installation & Usage

## Setting up the ESP32

1. Open `motor_control.ino` in the Arduino IDE.  
2. Ensure `PID.h` and `PID.cpp` are located in the same directory as your `.ino` sketch.  
3. Select your ESP32 board and the correct COM port.  
4. Compile and upload the code.

---

## Setting up the Dashboard

1. Ensure you have Python installed (3.8+ recommended).  
2. Install the required dependencies:

```bash
pip install streamlit pyserial pandas
```

3. Update the `SERIAL_PORT` variable in `dashboard.py` to match your ESP32's COM port:

- Windows: 'COM10' (example)

- Linux/Mac: '/dev/ttyUSB0'

Run the Streamlit app:
```
streamlit run dashboard.py
```
