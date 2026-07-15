# Crazyflie Flight Control, Telemetry, and Visualization

Python tools for controlling, testing, logging, visualizing, and replaying physical flights with the Crazyflie 2.x platform.

This repository contains the Python tools used to test the Crazyflie in real flight and support sim-to-real research. The code connects to the drone through a Crazyradio dongle using Bitcraze’s `cflib` library.

The project supports repeatable physical flight experiments by combining waypoint execution, configurable command rates, detailed telemetry collection, preflight safety checks, live trajectory visualization, and recorded-flight replay.

## Key Results

- Conducted more than **200 physical Crazyflie flight tests**
- Tested command rates from **10 to 100 Hz**
- Collected **361,142 telemetry packets** for flight-performance analysis
- Identified approximately **40 Hz** as the lower bound for consistently smooth waypoint control in the tested configuration
- Developed configurable logging for position, attitude, controller output, motor commands, and battery voltage
- Added live tracking-error visualization and a **two-second predicted flight trajectory**
- Created repeatable waypoint, manual-control, and replay workflows for analyzing sim-to-real flight behavior

## Project Development

The project includes:

- Development of the Crazyflie connection and telemetry client
- Design of repeatable physical waypoint-flight experiments
- Testing of how command frequency affects flight stability and trajectory tracking
- Configurable command and telemetry logging rates
- Battery monitoring, connection checks, arming, motor warmup, and emergency-stop behavior
- Recording of position, attitude, PID-controller output, motor commands, and battery telemetry
- Live flight visualization and recorded-flight replay
- Comparison of commanded and measured flight behavior
- Analysis of flight data to identify a practical lower command-rate boundary

This repository represents the physical flight-testing portion of a broader autonomous-flight project involving reinforcement learning, simulation, and sim-to-real deployment.

## Main Features

- Crazyradio connection and communication management
- Automated preflight and battery checks
- Configurable command rates from 5 to 100 Hz
- Configurable telemetry logging rates
- Waypoint-based position control
- Smooth movement between waypoint targets
- Keyboard-based manual hover control
- Live visualization of the estimated flight path
- Planned and actual trajectory comparison
- Current waypoint and target visualization
- Position and landing-error monitoring
- Two-second predicted trajectory display
- JSON flight-data export
- Recorded-flight replay
- Reference library for Crazyflie PID, estimator, motor, battery, and controller log variables

## Repository Structure

The main flight-control tools are located in:

```text
manual/cf2_flight_tools/
```

| File | Description |
|---|---|
| `cf2client.py` | Handles communication, connection callbacks, preflight checks, battery monitoring, telemetry logging, movement commands, motor warmup, and shutdown |
| `cf2flight.py` | Runs waypoint flights, controls command and logging rates, and saves recorded telemetry |
| `cf2manual_terminal.py` | Provides keyboard-based manual hover control from a Windows terminal |
| `cf2_flightmap.py` | Displays the live estimated path, planned trajectory, targets, tracking error, home position, and predicted motion |
| `cf2_replay_map.py` | Replays saved flight JSON files using the same visualization system |
| `pid_log_vars_reference.py` | Documents useful Crazyflie PID, estimator, motor, battery, and control variables |
| `requirements.txt` | Lists the required Python packages |
| `.gitignore` | Prevents generated caches, bytecode, and flight logs from being committed |

## Logged Flight Data

The flight system can record:

- Commanded position
- Estimated position
- Estimated velocity
- Estimated roll, pitch, and yaw
- Internal PID position targets
- Roll, pitch, yaw, and thrust controller outputs
- Individual motor requests
- Battery voltage
- Requested command rate
- Requested telemetry logging rate
- Takeoff and final position estimates
- Estimated landing offset
- Planned trajectory
- Time-ordered telemetry packets

Saved flight data can be used for plotting, controller analysis, command-rate testing, trajectory tracking evaluation, and sim-to-real debugging.

## Hardware

- Crazyflie 2.x
- Crazyradio PA or Crazyradio 2.0
- Flow Deck
- Z-Ranger Deck
- Windows computer

## Software

- Python
- Bitcraze `cflib`
- NumPy
- Tkinter
- Miniconda
- Git and GitHub

## Quick Start

Clone the repository and enter the flight-tools folder:

```powershell
git clone https://github.com/PelekanosAndreas/cf2dronelab-Damian-Andreas-.git
cd cf2dronelab-Damian-Andreas-\manual\cf2_flight_tools
```

Install the required packages:

```powershell
python -m pip install -r requirements.txt
```

Before running a physical flight, review the following settings inside `cf2flight.py`:

- Crazyflie radio URI
- Flight-area dimensions
- Waypoint positions
- Waypoint heights
- Waypoint durations
- Marker-deck settings
- Logged variables
- Command rate
- Logging rate
- Motor warmup settings

## Run a Waypoint Flight

> This command controls real hardware. Use a clear test area and remain ready to stop the drone.

```powershell
python cf2flight.py --command-hz 50 --log-hz 100
```

Disable the live map:

```powershell
python cf2flight.py --command-hz 50 --log-hz 100 --no-map
```

Choose an output folder:

```powershell
python cf2flight.py --command-hz 50 --log-hz 100 --output-dir outputs\test_01
```

Disable motor warmup:

```powershell
python cf2flight.py --motor-warmup-power 0
```

## Manual Terminal Control

The manual-control script uses Windows keyboard input and is intended for PowerShell or Command Prompt:

```powershell
python cf2manual_terminal.py --command-hz 50
```

Available controls are printed when the script starts.

Emergency stop and disarm:

```text
Space or X
```

## Replay a Recorded Flight

Replay a recorded flight:

```powershell
python cf2_replay_map.py outputs\flight_file.json
```

Replay at twice the original speed:

```powershell
python cf2_replay_map.py outputs\flight_file.json --speed 2
```

## Current Scope

The current version uses the Crazyflie's onboard position estimate for live visualization and flight analysis.

Motion-capture marker settings are included as early configuration hooks, but full motion-capture integration is not active in the current version.

## Safety

- Test in a clear and controlled flight area
- Keep people away from the propellers
- Confirm battery voltage before takeoff
- Verify the radio URI and waypoint limits
- Keep an emergency-stop method available
- Begin with conservative heights and speeds
- Review all flight parameters before running `cf2flight.py`

---

# Windows Crazyflie Setup

This guide explains how to set up a Windows machine to run a Crazyflie Python flight script using the Crazyradio dongle.

This setup is for **Windows PowerShell / Miniconda**, not WSL/Ubuntu.

## Important Notes

- `pip` installs the Python library: `cflib`.
- **libusbK** is the USB driver Windows needs for the Crazyradio dongle.
- **Zadig** is the tool used to install the `libusbK` driver onto the Crazyradio dongle.
- **Homebrew is not needed on Windows.**

## 1. Install Python Dependencies

Open PowerShell or Anaconda Prompt and run:

```powershell
pip install --upgrade pip
pip install --upgrade cflib pyusb libusb-package
```

## 2. Download libusbK

Download libusbK for Windows:

```text
https://sourceforge.net/projects/libusbk/
```

Install libusbK on the computer.

This gives Windows the USB driver package needed for Crazyradio access.

## 3. Install libusbK onto the Crazyradio Dongle Using Zadig

Plug in the **Crazyradio dongle**.

Download and open Zadig:

```text
https://zadig.akeo.ie/
```

Use Zadig to assign the **libusbK** driver to the Crazyradio dongle:

1. Click `Options`.
2. Enable `List All Devices`.
3. Select the Crazyradio device. It may appear as:
   - `Crazyradio PA USB Dongle`
   - `Crazyradio 2.0`
   - `Bitcraze Crazyradio`
4. In the driver selection box, choose:

```text
libusbK (v3.1.0.0)
```

The displayed version may differ.

5. Click `Install Driver` or `Replace Driver`.
6. Wait for the driver installation to finish.
7. Unplug and reconnect the Crazyradio dongle.

After this step, Windows should allow Python and `cflib` to access the Crazyradio.

## 4. Verify Crazyradio Detection

Run this in PowerShell:

```powershell
python -c "import cflib.crtp; cflib.crtp.init_drivers(); print(cflib.crtp.scan_interfaces())"
```

If the driver is not working, the output may look like:

```text
Cannot find a Crazyradio Dongle
[]
```

If the driver is working, the output should include a radio interface, for example:

```text
[['radio://0/80/2M', '']]
```

## 5. Test the Client Code Setup

This setup verifies that the Windows machine can run the Crazyflie client-side Python code and detect the Crazyradio dongle.

This step does **not** run a flight script.

From PowerShell, go to the cloned project folder:

```powershell
cd path\to\your\cloned\repository
```

Check that Python can import the Crazyflie client library:

```powershell
python -c "import cflib; print('cflib import OK')"
```

Check that Python can detect the Crazyradio dongle:

```powershell
python -c "import cflib.crtp; cflib.crtp.init_drivers(); print(cflib.crtp.scan_interfaces())"
```

A working result should show a radio interface, for example:

```text
[['radio://0/80/2M', '']]
```

## Troubleshooting

### Error: `Cannot find a Crazyradio Dongle`

This usually means Windows or Python cannot access the Crazyradio USB dongle.

Try the following:

1. Unplug and reconnect the Crazyradio.
2. Reopen Zadig and confirm the selected device is the **Crazyradio dongle**, not the drone.
3. Confirm that the installed driver is:

```text
libusbK
```

4. If needed, click `Replace Driver` in Zadig and reinstall `libusbK`.
5. Try a different USB port.
6. Avoid USB hubs.
7. Close any other Crazyflie client or program that may already be using the dongle.
8. Run the detection command again:

```powershell
python -c "import cflib.crtp; cflib.crtp.init_drivers(); print(cflib.crtp.scan_interfaces())"
```

### Output is `[]`

If the output is:

```text
[]
```

then no Crazyradio interface was detected.

The expected working output should look similar to:

```text
[['radio://0/80/2M', '']]
```

## Sources

Bitcraze Windows USB driver documentation:

```text
https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/
```

Bitcraze `cflib` installation documentation:

```text
https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/install/
```

## Acknowledgments

Developed through autonomous-flight research with the LEADCAT Research Group at the University of Illinois Urbana-Champaign.

Project contributors include Andreas Pelekanos and Damian Thomas.
