"""
Flight script for connecting to a Crazyflie, collecting telemetry logs, and
writing the recorded flight data to a JSON file after the run.
"""

############# IMPORTS #############
import argparse
import time
import json
from pathlib import Path
from cf2client import CF2Client
from cf2_flightmap import CF2FlightMap

 
############# PARAMETERS #############
parser = argparse.ArgumentParser(description='Run a Crazyflie waypoint flight.')
parser.add_argument('--log-hz', type=float, default=100, help='Telemetry logging rate per log config.')
parser.add_argument('--command-hz', type=float, default=20, help='Position setpoint command rate.')
parser.add_argument('--no-map', action='store_true', help='Disable the live flight map window.')
parser.add_argument('--idle-thrust', type=int, default=3000, help='powerDist.idleThrust value. Use 0 to disable setting it.')
parser.add_argument('--motor-warmup-power', type=int, default=5000, help='Direct motor power used during warmup. Use 0 to disable warmup.')
parser.add_argument('--motor-warmup-duration', type=float, default=5.5, help='Seconds to hold warmup motor power after ramp-up.')
parser.add_argument('--motor-warmup-ramp-time', type=float, default=1.5, help='Seconds to ramp motors up before the warmup hold.')
parser.add_argument('--output-dir', type=Path, default=Path('outputs'), help='Directory for saved flight JSON files.')
args = parser.parse_args()

if args.log_hz < 1 or args.log_hz > 100:
    raise ValueError('--log-hz must be between 1 and 100 Hz.')
if args.command_hz < 5 or args.command_hz > 100:
    raise ValueError('--command-hz must be between 5 and 100 Hz.')
if args.idle_thrust < 0 or args.idle_thrust > 8000:
    raise ValueError('--idle-thrust must be between 0 and 8000.')
if args.motor_warmup_power < 0 or args.motor_warmup_power > 20000:
    raise ValueError('--motor-warmup-power must be between 0 and 20000.')
if args.motor_warmup_duration < 0:
    raise ValueError('--motor-warmup-duration must be greater than or equal to 0.')
if args.motor_warmup_ramp_time < 0:
    raise ValueError('--motor-warmup-ramp-time must be greater than or equal to 0.')


def build_planned_trajectory(waypoints, samples_per_second):
    planned = []
    if not waypoints:
        return planned

    previous_x, previous_y, previous_z = waypoints[0][:3]
    elapsed_s = 0.0

    for x, y, z, duration_s in waypoints:
        sample_count = max(2, int(round(duration_s * samples_per_second)))
        for sample_index in range(sample_count):
            fraction = sample_index / (sample_count - 1)
            planned_x = previous_x + (x - previous_x) * fraction
            planned_y = previous_y + (y - previous_y) * fraction
            planned_z = previous_z + (z - previous_z) * fraction
            planned.append((planned_x, planned_y, planned_z, elapsed_s + duration_s * fraction))

        previous_x, previous_y, previous_z = x, y, z
        elapsed_s += duration_s

    return planned


# Specify the URI of the drone to connect to.
uri = 'radio://0/80/2M/E7E7E7E7E7'

# Flight-area and waypoint settings.
# The drone starts in the center of a 4 ft x 4 ft square.
FT_TO_M = 0.3048
flight_area_side_ft = 4.0
flight_area_half_width_m = (flight_area_side_ft * FT_TO_M) / 2.0
waypoint_altitude_m = 0.20
landing_step_altitudes_m = [0.15, 0.10, 0.05]
square_step_m = 0.30
takeoff_hover_duration_s = 2.0
waypoint_duration_s = 2.0
landing_step_duration_s = 1.2

# Each waypoint is (x_m, y_m, z_m, duration_s), relative to the center of the
# 4 ft x 4 ft square. Keep x/y within +/- flight_area_half_width_m.
takeoff_waypoints = [
    (0.0, 0.0, waypoint_altitude_m, takeoff_hover_duration_s),
]

square_waypoints = [
    (square_step_m, 0.0, waypoint_altitude_m, waypoint_duration_s),
    (square_step_m, square_step_m, waypoint_altitude_m, waypoint_duration_s),
    (0.0, square_step_m, waypoint_altitude_m, waypoint_duration_s),
    (0.0, 0.0, waypoint_altitude_m, waypoint_duration_s),
]

landing_waypoints = [
    (0.0, 0.0, altitude_m, landing_step_duration_s)
    for altitude_m in landing_step_altitudes_m
]

waypoints = takeoff_waypoints + square_waypoints + landing_waypoints

# Specify the name of the rigid body that corresponds to your active marker
# deck in the motion capture system. If your marker deck number is X, this name
# should be 'marker_deck_X'.
marker_deck_name = 'marker_deck_70'

# Specify the marker IDs that correspond to your active marker deck in the
# motion capture system. If your marker deck number is X, these IDs should be
# [X + 1, X + 2, X + 3, X + 4]. They are listed in clockwise order (viewed
# top-down), starting from the front.
marker_deck_ids = [71, 72, 73, 74]

# Specify whether or not to use the motion capture system
use_mocap = False

# Output location: outputs/<experiment_subject>/<test_name>/data/<timestamp>.json
experiment_subject = 'experiment_subject'
test_name = 'test'

# Logging settings
log_hz = args.log_hz

# Command settings
command_hz = args.command_hz

#Specify the variables you want to log from the drone

variables = [
# ctrltarget.x/y/z = commanded position setpoint from the commander 
# Did my command arrive correctly?
'ctrltarget.x', 
'ctrltarget.y', 
'ctrltarget.z',

# stateEstimate.x/y/z  = estimated actual drone position
# Is the drone estimating its position correctly?
 'stateEstimate.x', 
 'stateEstimate.y', 
 'stateEstimate.z',

# Estimated roll, pitch, yaw (sensors)
'stateEstimate.roll', 
'stateEstimate.pitch', 
'stateEstimate.yaw',

# posCtl.targetX/Y/Z  = internal position-controller target used by PID
# Is the PID controller using the target I expect?
'posCtl.targetX', 
'posCtl.targetY', 
'posCtl.targetZ',

# Final PID controller outputs sent toward the motor mixer
'controller.cmd_thrust',
'controller.cmd_roll',
'controller.cmd_pitch', 
'controller.cmd_yaw',

# #Individual motor output requests after mixing (motor power command)
# Is one motor working harder than the others?
# Are motors saturating?
# Is the mixer producing reasonable outputs?
# Is the drone compensating unevenly?

'motor.m1req', 
'motor.m2req', 
'motor.m3req', 
'motor.m4req',

# Battery telemetry for post-flight checks.
'pm.vbat',
'pm.vbatMV',
]


############# FLIGHT CODE #############
for x, y, z, duration in waypoints:
    if abs(x) > flight_area_half_width_m or abs(y) > flight_area_half_width_m:
        raise ValueError(
            f'Waypoint ({x}, {y}, {z}) is outside the '
            f'{flight_area_side_ft:g} ft x {flight_area_side_ft:g} ft square.'
        )
    if z < 0.0:
        raise ValueError(f'Waypoint z must be non-negative: ({x}, {y}, {z})')

planned_trajectory = build_planned_trajectory(waypoints, command_hz)

flight_map = None
if not args.no_map:
    flight_map = CF2FlightMap(flight_area_half_width_m=flight_area_half_width_m)
    flight_map.set_waypoints(waypoints)
    flight_map.set_planned_trajectory(planned_trajectory)
    flight_map.start()

# Create and start the client that will connect to the drone
drone_client = CF2Client(
    uri,
    log_variables=variables,
    marker_deck_ids = marker_deck_ids if use_mocap else None,
    log_mode="memory",
    log_hz=log_hz,
    battery_check_timeout=8.0,
    live_map=flight_map,
    idle_thrust=args.idle_thrust if args.idle_thrust > 0 else None,
    motor_warmup_power=args.motor_warmup_power,
    motor_warmup_duration=args.motor_warmup_duration,
)

# Wait until the client is fully connected to the drone
connect_deadline = time.time() + 20
while not drone_client.is_fully_connected:
    if time.time() > connect_deadline:
        if flight_map is not None:
            flight_map.close()
        drone_client.disconnect()
        raise TimeoutError(f'Crazyflie did not become ready within 20 seconds: {uri}')
    time.sleep(0.1)


# Flight Path
command_count = 0
command_start_time_s = None
command_end_time_s = None
battery_after = None

try:
    drone_client.motor_idle_test(
        ramp_time=args.motor_warmup_ramp_time,
        ramp_down=False,
    )
    for waypoint_index, (x, y, z, duration) in enumerate(waypoints):
        print(f'Waypoint: x={x:.2f} m, y={y:.2f} m, z={z:.2f} m for {duration:.1f} s')
        if command_start_time_s is None:
            command_start_time_s = time.time()
        if waypoint_index == 0:
            command_count += drone_client.move(
                x,
                y,
                z,
                yaw=0.0,
                duration=duration,
                command_dt=1.0 / command_hz,
            )
        else:
            command_count += drone_client.move_smooth_to(
                x,
                y,
                z,
                yaw=0.0,
                duration=duration,
                command_dt=1.0 / command_hz,
            )
        command_end_time_s = time.time()
    time.sleep(1)
finally:
    # Always stop commands and close the link, even if the flight block errors.
    try:
        drone_client.stop()
        battery_after = drone_client.read_battery_snapshot()
        drone_client.battery_after = battery_after
        if battery_after is not None:
            after_parts = []
            if battery_after.get("voltage_v") is not None:
                after_parts.append(f"{battery_after['voltage_v']:.2f} V")
            print(
                "Battery after flight: "
                f"{' / '.join(after_parts) if after_parts else 'telemetry received'}"
            )
        home_error = drone_client.estimated_home_error()
        if home_error is not None:
            print(
                "Estimated landing offset from takeoff: "
                f"xy={home_error['xy_m'] * 100:.1f} cm, "
                f"z={abs(home_error['dz_m']) * 100:.1f} cm"
            )
    finally:
        drone_client.disconnect()





############# Output Parameters #############
# Memory mode stores telemetry in RAM; package it here for post-flight JSON export.
# CSV/JSON modes write during flight, so this export block is mainly for memory mode.

if drone_client.log_mode == "memory":
    if drone_client.log_records:
        if command_start_time_s is not None and command_end_time_s is not None:
            command_elapsed_s = command_end_time_s - command_start_time_s
            if command_elapsed_s > 0:
                measured_command_hz = command_count / command_elapsed_s
                print("")
                print("Command Hz summary")
                print(f"  command requested: {command_hz:g} Hz")
                print(f"  command measured:  {measured_command_hz:.1f} Hz")
                print(f"  command count:     {command_count}")
                print(f"  command window:    {command_elapsed_s:.2f} s")

        first_time_s = drone_client.log_records[0]["host_time_s"]
        last_time_s = drone_client.log_records[-1]["host_time_s"]
        elapsed_s = last_time_s - first_time_s
        log_config_count = max(1, len(set(record["logconf"] for record in drone_client.log_records)))
        if elapsed_s > 0:
            total_log_hz = len(drone_client.log_records) / elapsed_s
            per_config_log_hz = total_log_hz / log_config_count
            print("")
            print("Log Hz summary")
            print(f"  log requested/config: {log_hz:g} Hz")
            print(f"  log measured/config:  {per_config_log_hz:.1f} Hz")
            print(f"  log packets total:    {total_log_hz:.1f} Hz")
            print(f"  log configs:          {log_config_count}")
            print(f"  log window:           {elapsed_s:.2f} s")

    measured_command_hz = None
    command_elapsed_s = None
    if command_start_time_s is not None and command_end_time_s is not None:
        command_elapsed_s = command_end_time_s - command_start_time_s
        if command_elapsed_s > 0:
            measured_command_hz = command_count / command_elapsed_s

    data = {
        'flight_settings': {
            'uri': uri,
            'command_hz': command_hz,
            'log_hz': log_hz,
            'flight_area_side_ft': flight_area_side_ft,
            'flight_area_half_width_m': flight_area_half_width_m,
            'use_mocap': use_mocap,
            'marker_deck_name': marker_deck_name if use_mocap else None,
            'marker_deck_ids': marker_deck_ids if use_mocap else None,
            'idle_thrust': args.idle_thrust,
            'motor_warmup_power': args.motor_warmup_power,
            'motor_warmup_duration': args.motor_warmup_duration,
            'motor_warmup_ramp_time': args.motor_warmup_ramp_time,
        },
        'command_summary': {
            'requested_hz': command_hz,
            'measured_hz': measured_command_hz,
            'command_count': command_count,
            'command_window_s': command_elapsed_s,
        },
        'waypoints': waypoints,
        'battery_before': drone_client.battery_before,
        'battery_after': drone_client.battery_after,
        'takeoff_position_estimate': drone_client.takeoff_position,
        'final_position_estimate': drone_client.current_position,
        'estimated_home_error': drone_client.estimated_home_error(),
        'planned_trajectory': planned_trajectory,
        'drone': {
            # Every log packet in the order it arrived from the Crazyflie.
            'records': drone_client.log_records,
            # Same telemetry grouped by variable name, best format for plotting.
            'by_variable': drone_client.log_data,
            # Most recent value received for each logged variable.
            'latest_values': drone_client.latest_log_values,
        }
    }


    # Write flight data to a file
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    output_path = args.output_dir / f'{timestamp}.json'
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w') as outfile:
        json.dump(data, outfile, sort_keys=False)

    print(f'Wrote flight data to {output_path}')
# else:
#     print(f'Skipped post-flight JSON export because log_mode={drone_client.log_mode!r}.')

if flight_map is not None:
    input("Flight map is still open. Press Enter to close it...")
    flight_map.close()
