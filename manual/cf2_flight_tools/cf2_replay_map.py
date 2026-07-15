import argparse
import json
import time
from pathlib import Path

from cf2_flightmap import CF2FlightMap


def build_rate_text(data):
    settings = data.get("flight_settings", {})
    command_hz = settings.get("command_hz")
    if command_hz is not None:
        return f"command requested {command_hz:g} Hz"

    return "command Hz unavailable"


parser = argparse.ArgumentParser(description="Replay a Crazyflie flight JSON on the live map.")
parser.add_argument("flight_json", type=Path, help="Flight JSON written by cf2flight.py.")
parser.add_argument("--speed", type=float, default=2.0, help="Replay speed multiplier.")
args = parser.parse_args()

if args.speed <= 0:
    raise ValueError("--speed must be greater than 0.")

with open(args.flight_json) as input_file:
    data = json.load(input_file)

settings = data.get("flight_settings", {})
flight_area_half_width_m = settings.get("flight_area_half_width_m", 0.6096)
rate_text = build_rate_text(data)
title = f"CF2 Flight Replay: {args.flight_json.name}"
if rate_text:
    title = f"{title}\n{rate_text}"

flight_map = CF2FlightMap(
    flight_area_half_width_m=flight_area_half_width_m,
    title=title,
)
flight_map.set_waypoints(data.get("waypoints", []))
flight_map.set_planned_trajectory(data.get("planned_trajectory", []))

takeoff_position = data.get("takeoff_position_estimate")
if takeoff_position is not None:
    flight_map.set_home(
        takeoff_position["x"],
        takeoff_position["y"],
        takeoff_position["z"],
    )

flight_map.start()

records = data.get("drone", {}).get("records", [])
latest = {}
previous_time_s = None

try:
    for record in records:
        values = record.get("values", {})
        latest.update(values)

        host_time_s = record.get("host_time_s")
        if previous_time_s is not None and host_time_s is not None:
            delay_s = max(0.0, host_time_s - previous_time_s) / args.speed
            time.sleep(min(delay_s, 0.2))
        previous_time_s = host_time_s

        if all(variable in latest for variable in ("ctrltarget.x", "ctrltarget.y", "ctrltarget.z")):
            flight_map.set_target(
                latest["ctrltarget.x"],
                latest["ctrltarget.y"],
                latest["ctrltarget.z"],
            )

        if all(variable in latest for variable in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z")):
            flight_map.update_position(
                latest["stateEstimate.x"],
                latest["stateEstimate.y"],
                latest["stateEstimate.z"],
                host_time_s,
                latest.get("stateEstimate.vx"),
                latest.get("stateEstimate.vy"),
                latest.get("stateEstimate.vz"),
            )

    input("Replay complete. Press Enter to close the map...")
finally:
    flight_map.close()
