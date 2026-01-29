# Optisands Biomimicry

This repository contains the full software stack developed for our biomimetic research robot designed for coastal and beach environments. The robot is intended to autonomously collect environmental data such as:

- Air temperature and humidity

- Sand grain distribution based on size

The software integrates multiple sensors, motor control systems, a PlayStation controller interface and a computer-vision pipeline running on a Raspberry Pi 5.

## Installation

### Cloning the Repository Using SSH
This repository is accessed via SSH authentication. Before cloning, make sure you have an SSH key configured on your system and added to your GitHub account.


### Libraries and Dependencies
1. Standard Python Libraries
These are built into Python that were used:

- time, sleep
- random
- sys
- os
- threading
- csv
- datetime


2. Numerical and Plotting Libraries
Used for data processing and visualization:

- numpy
- matplotlib.pyplot


3. Raspberry Pi & Hardware Libraries
These libraries interface with sensors, motors and communication hardware:

- RPi.GPIO
- gpiozero
- pigpio / python3-pigpio
- smbus / SMBus
- board
- busio
- serial (pyserial)

Sensor-specific libraries:

- adafruit_bno055
- adafruit_sht31d
- adafruit-circuitpython-gps
- DFRobot_RaspberryPi_A02YYUW (clone gitHub repository)


4. Controller
Used for manual operation:

- pyPS4Controller

5. Computer Vision
Used for image capture and shoreline analysis:

- cv2 (OpenCV)
- rpicam-apps

6. Environment Setup
A Python virtual environment is used:


```bash
sudo apt-get update
sudo apt-get install -y python3-venv
python3 -m venv rosenv
source rosenv/bin/activate
```

Some hardware-specific libraries are installed directly on Raspberry Pi OS.



## Repository Structure
The repository is organized to separate experimental code from the final operational system.

### DFRobot A02YYUW library submodule
The ultrasonic water-level sensor (DFRobot A02YYUW) does not provide a package that can be installed directly via pip. To use this sensor, its Python library must be cloned manually from the manufacturer’s GitHub repository.

```bash
sudo git clone https://github.com/DFRobot/DFRobot_RaspberryPi_A02YYUW
```

### Testing - Testing & unused scripts/

Sensors/
Contains early scripts for connecting and testing all onboard sensors.
These files represent the first stage of integrating hardware into the software.


Vision/
This folder documents development of the computer-vision subsystem.


Radio/
Scripts created for testing radio communication.
These were not used in the final prototype but remain for future expansion.

Motor & Controller/
These helped tune the motor drivers and the controller.


### Final System Folder – system/
This folder contains the final production-ready scripts used on the deployed robot:

- Humidity and temperature sensor integration
- PlayStation controller interface with motor control software
- PlayStation controller interface without motor control software
- Image capture pipeline
- Result files

These files were executed during real-world field experiments.


## Useful Commands for Setup and Testing

1. Virtual Environment Activation

```bash
source rosenv/bin/activate
```

2. Camera Testing Commands

```bash
rpicam-still -o image.jpg
rpicam-hello -t 0
```

3. I2C Device Detection

```bash
i2cdetect -y 1
```

4. Checking Running Python Processes
When working with hardware sensors, it is important to make sure that a sensor is not already being accessed by another script.
Running multiple programs that use the same GPIO or I2C device at the same time can cause conflicts or crashes.

To list all running Python processes:

```bash
ps aux | grep python
```

To terminate a process:

```bash
kill -9 PID
```