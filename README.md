# Capstone-II-Mass-Adjustable-RC-Car-Project

This repository contains the control, sensor, logging, and analysis code for the Northeastern University Mass Adjustable RC Car Capstone II project, advised by Professor Carlos Hidrovo.

Full technical document:
https://docs.google.com/document/d/14RuH56wbCEfM_qvJvr-gffSnfkfuiTqH0ljx36ycuP4/edit?usp=sharing

This README is the short, practical setup guide for working in this repository. The technical document still holds the detailed wiring diagrams, long-form hardware notes, and reference material.

## What Is In This Repo

- `Controller.py` is the main runtime loop. It calibrates the IMU on startup, reads steering data from the Arduino Nano, updates odometry, and logs a CSV file to `data/`.
- `IMU.py` and `Odometry.py` handle the BNO055 IMU and derived motion estimates.
- `RCReceiverNano.py` reads `$STEER,...` messages from the Arduino Nano over serial, usually `/dev/ttyUSB0`.
- `GPS_System.py` wraps `gpsd` and exposes the latest GPS fix for the controller.
- `Plotting.py` plots every numeric column in `data/data.csv`.
- `Arduino Sketches/` contains the Nano firmware used for RC receiver testing and passthrough.
- `tools/bwsi-python.cmd` lets Windows users run scripts through the expected `bwsi` conda environment without activating it first.

## Expected Hardware

The current codebase is built around:

- Raspberry Pi 4 Model B
- Adafruit BNO055 IMU over I2C
- Arduino Nano sending steering data over USB serial
- Optional GPS HAT configured through `gpsd`
- Optional camera and GPIO-driven peripherals

## Quick Start

### Windows / laptop workflow

Use this path for editing the code, plotting logged data, and lightweight development. Hardware-facing scripts still need a Raspberry Pi with the attached sensors.

1. Install Anaconda to the default location at `%USERPROFILE%\anaconda3`.
2. Create the conda environment expected by the repo:

   ```powershell
   conda create -n bwsi python=3.13 anaconda
   conda activate bwsi
   conda install -c conda-forge utm
   pip install -r requirements.txt
   ```

3. Run local scripts through the wrapper:

   ```powershell
   .\tools\bwsi-python.cmd Plotting.py
   ```

4. For headless plotting from PowerShell:

   ```powershell
   $env:MPLBACKEND='Agg'
   .\tools\bwsi-python.cmd Plotting.py
   ```

5. VS Code includes matching tasks:
   - `Python: Run active file in bwsi`
   - `Python: Run active file in bwsi (headless)`

`tools\bwsi-python.cmd` assumes either `conda.exe` or `python.exe` exists inside `%USERPROFILE%\anaconda3` and that the environment is named `bwsi`.

### Raspberry Pi / robot workflow

Use this path for IMU, serial receiver, GPS, camera, and GPIO development.

1. Flash Raspberry Pi OS with Raspberry Pi Imager.
2. Set a hostname, enable SSH, and preconfigure Wi-Fi if needed.
   The technical document uses `carpi` as the example hostname.
3. Use your own username and password rather than reusing the example credentials from the original technical document.
4. SSH into the Pi:

   ```bash
   ssh pi@carpi.local
   ```

5. If the host key changed after reflashing:

   ```bash
   ssh-keygen -R carpi.local
   ```

6. Update the system:

   ```bash
   sudo apt update
   sudo apt full-upgrade -y
   ```

7. Enable the interfaces this project uses:

   ```bash
   sudo raspi-config
   ```

   Enable:
   - `SSH`
   - `Camera`
   - `I2C`
   - `Serial Port`

   For the serial prompt inside `raspi-config`, choose:
   - `No` for login shell over serial
   - `Yes` to keep the serial hardware enabled

8. Install base system packages:

   ```bash
   sudo apt install -y python3 python3-pip python3-venv python3-full python3-rpi.gpio gpsd gpsd-clients pigpio
   ```

9. Create and activate a virtual environment in the repo:

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install --upgrade pip setuptools
   pip install -r requirements.txt
   ```

10. Install Blinka / CircuitPython support for Raspberry Pi so the BNO055 library can talk to the hardware:

   ```bash
   cd ~
   sudo pip3 install --upgrade adafruit-python-shell --break-system-packages
   wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/raspi-blinka.py
   sudo python3 raspi-blinka.py
   sudo reboot
   ls /dev/i2c* /dev/spi*
   ```

11. If you plan to use `GPS_System.py`, also install the Python gpsd client that provides `from gps import gps`, then configure `/etc/default/gpsd` and verify the receiver with:

   ```bash
   # /etc/default/gpsd
   DEVICES="/dev/serial0"
   GPSD_OPTIONS="-n"
   USBAUTO="true"

   sudo reboot
   cgps
   gpsmon
   sudo ppstest /dev/pps0
   ```

## Running The Code

### Main logger

Run the controller on the Pi after the IMU, Arduino Nano, and optional GPS hardware are connected:

```bash
source .venv/bin/activate
python Controller.py test_run True
```

Argument order for `Controller.py`:

- First argument: output filename stem
- Second argument: verbose flag (`True` or `False`)

Example:

- `python Controller.py imu_session True` writes `data/imu_session.csv`

### Hardware bring-up scripts

These modules can be run directly on the Pi while bringing hardware online:

```bash
python IMU.py
python Odometry.py
python RCReceiverNano.py
python GPS_System.py
```

### Plotting a logged run

`Plotting.py` currently reads `./data/data.csv` and expects a `time` column. If you want to plot another run, either update `file_path` inside `Plotting.py` or rename/copy the target CSV to `data/data.csv`.

## Bring-Up Notes

### IMU calibration

The IMU code performs an interactive BNO055 calibration sequence. The startup prompts match the workflow from the technical document:

- Gyroscope: keep the sensor still
- Magnetometer: move the sensor normally or in figure-eight motion
- Accelerometer: place the sensor in six stable orientations

If the IMU reports poor calibration, the orientation and derived odometry values will not be reliable.

### RC receiver voltage safety

Before wiring the RC receiver PWM output into the Pi, verify the signal is around `3.3V` max. If the receiver outputs around `6V`, use a voltage divider or level shifting before connecting it to the Raspberry Pi GPIO input. The original guide uses `R1 = 2.7 kOhm` and `R2 = 3.3 kOhm` as one example divider.

### Serial device name

The Arduino Nano usually appears as:

```bash
ls /dev/ttyUSB*
```

In the current code, `RCReceiverNano.py` defaults to `/dev/ttyUSB0`.

### GPIO and pin inspection

Useful Raspberry Pi checks from the original setup guide:

```bash
libcamera-still -o libcamera_test.jpg
pinout
sudo usermod -a -G gpio pi
```

If you change the Pi username, replace `pi` in the `usermod` command.

## Known Repo Assumptions

- `requirements.txt` captures the repo's direct Python dependencies, but `GPS_System.py` still relies on gpsd and its Python bindings being installed through the OS package manager on the Pi.
- The technical document uses the hostname `carpi`, but the hardware import guards in `Controller.py`, `IMU.py`, and `Odometry.py` currently use hardcoded hostname checks. Verify or update those checks before first run on a newly named Pi.
- `Plotting.py` is designed for quick inspection rather than a full analysis pipeline.

## Useful Files

- `data/data.csv` contains an example log file.
- `.vscode/tasks.json` wires the Windows runner into VS Code tasks.
- `requirements.txt` is the repo's Python dependency list.
- `tools/bwsi-python.cmd` is the easiest way to run local scripts in the `bwsi` environment.

## Need More Detail?

Use the technical document for:

- Wiring diagrams and assembly notes
- Detailed Raspberry Pi flashing steps
- GPS HAT setup details
- RC receiver validation notes and Arduino-side testing
- Hardware datasheets and external references
