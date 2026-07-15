# Crazyflie 2.0 Flight Tools

Python tools for connecting to a Crazyflie 2.x, running a waypoint flight with the default PID controller, logging telemetry, displaying a live 2D flight map, replaying saved flights, and manually controlling hover commands from a Windows terminal.

## Files

- `cf2client.py`: Crazyflie connection, preflight checks, telemetry logging, battery checks, motor warmup, movement commands, and log export.
- `cf2flight.py`: Configurable waypoint flight that records telemetry to JSON.
- `cf2_flightmap.py`: Tkinter live map with target, trail, home position, and predicted motion.
- `cf2_replay_map.py`: Replays a JSON flight log on the same map.
- `cf2manual_terminal.py`: Windows keyboard control using hover setpoints.
- `pid_log_vars_reference.py`: Reference list of useful default PID and estimator log variables.

## Requirements

- Python 3.10 or newer
- Crazyradio PA or compatible Crazyflie radio
- Crazyflie Python library (`cflib`)
- NumPy
- `libusb-package` on Windows for the manual terminal script
- Tkinter for the live and replay maps

Install the Python packages in the active project environment:

```powershell
python -m pip install cflib numpy libusb-package
```

## Before flying

Update the `uri`, waypoint values, flight area, and marker-deck settings in `cf2flight.py`. Test with the Crazyflie in a clear area and be ready to stop the flight. The scripts control real hardware.

## Run a waypoint flight

From this folder:

```powershell
python cf2flight.py --command-hz 50 --log-hz 100
```

Disable the map:

```powershell
python cf2flight.py --command-hz 50 --log-hz 100 --no-map
```

Choose a different output directory:

```powershell
python cf2flight.py --output-dir outputs/test_01
```

## Replay a saved flight

```powershell
python cf2_replay_map.py outputs/20260714_220000.json
```

Use `--speed` to change replay speed:

```powershell
python cf2_replay_map.py outputs/20260714_220000.json --speed 2
```

## Manual terminal control

This script currently uses `msvcrt`, so it is intended for Windows PowerShell or Command Prompt:

```powershell
python cf2manual_terminal.py --command-hz 50
```

Controls are printed when the program starts. Press `space` or `x` for an emergency stop and disarm.

## Generated files

Do not commit Python bytecode, Crazyflie cache metadata, or flight output logs unless a specific dataset belongs in the repository. Recommended ignore rules are included in `.gitignore`.
