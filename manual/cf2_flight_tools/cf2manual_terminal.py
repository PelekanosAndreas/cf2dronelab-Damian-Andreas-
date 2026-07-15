"""
Manual terminal hover control for a Crazyflie.

This uses the Crazyflie hover commander instead of raw thrust:
    vx [m/s], vy [m/s], yaw rate [deg/s], target height z [m]

Keys:
    r/f      target height up/down
    w/s      forward/back velocity
    a/d      left/right velocity
    q/e      yaw left/right
    c        stop horizontal/yaw motion
    l        land gently
    space/x  emergency stop and disarm
    ?        show controls
"""

import os
import time
import argparse
from pathlib import Path

try:
    import libusb_package

    libusb_dir = str(Path(libusb_package.__file__).resolve().parent)
    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(libusb_dir)
    os.environ["PATH"] = libusb_dir + os.pathsep + os.environ.get("PATH", "")
except Exception:
    pass

import msvcrt
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils.power_switch import PowerSwitch


URI = "radio://0/80/2M/E7E7E7E7E7"

HEIGHT_STEP_M = 0.03
VELOCITY_STEP_M_S = 0.05
YAW_STEP_DPS = 15.0

MIN_HEIGHT_M = 0.0
MAX_HEIGHT_M = 0.50
MAX_VELOCITY_M_S = 0.30
MAX_YAW_DPS = 60.0

DEFAULT_COMMAND_HZ = 50.0
RAMP_PER_SECOND = 2.0
LAND_RATE_M_S = 0.08


def parse_args():
    parser = argparse.ArgumentParser(description="Manual hover keyboard control for a Crazyflie.")
    parser.add_argument("--uri", default=URI)
    parser.add_argument("--no-reboot", action="store_true", help="Connect without rebooting the Crazyflie first.")
    parser.add_argument("--command-hz", type=float, default=DEFAULT_COMMAND_HZ, help="Hover command rate in Hz.")
    return parser.parse_args()


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def approach(current, target, max_delta):
    if current < target:
        return min(current + max_delta, target)
    if current > target:
        return max(current - max_delta, target)
    return current


def arm(cf, do_arm):
    if hasattr(cf, "supervisor"):
        cf.supervisor.send_arming_request(do_arm)
    else:
        cf.platform.send_arming_request(do_arm)


def stop_now(cf):
    cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.0)
    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()
    arm(cf, False)


def print_controls():
    print()
    print("Manual hover controls")
    print("  r/f      target height up/down")
    print("  w/s      forward/back velocity")
    print("  a/d      left/right velocity")
    print("  q/e      yaw left/right")
    print("  c        stop horizontal/yaw motion")
    print("  l        land gently")
    print("  space/x  emergency stop and disarm")
    print()


def main():
    args = parse_args()
    if args.command_hz < 5 or args.command_hz > 100:
        raise ValueError("--command-hz must be between 5 and 100 Hz.")
    command_period_s = 1.0 / args.command_hz

    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache="./__cfcache__")

    state = {
        "ready": False,
        "closed": False,
        "vx_target": 0.0,
        "vy_target": 0.0,
        "yawrate_target": 0.0,
        "height_target": 0.0,
        "vx": 0.0,
        "vy": 0.0,
        "yawrate": 0.0,
        "height": 0.0,
        "landing": False,
    }

    def connected(uri):
        print(f"CONNECTED: {uri}")

    def fully_connected(uri):
        print(f"FULLY_CONNECTED: {uri}")
        state["ready"] = True

    def failed(uri, msg):
        print(f"CONNECTION_FAILED: {uri}: {msg}")
        state["closed"] = True

    def lost(uri, msg):
        print(f"CONNECTION_LOST: {uri}: {msg}")
        state["closed"] = True

    def disconnected(uri):
        print(f"DISCONNECTED: {uri}")
        state["closed"] = True

    cf.connected.add_callback(connected)
    cf.fully_connected.add_callback(fully_connected)
    cf.connection_failed.add_callback(failed)
    cf.connection_lost.add_callback(lost)
    cf.disconnected.add_callback(disconnected)

    if not args.no_reboot:
        print(f"Rebooting Crazyflie at {args.uri}")
        PowerSwitch(args.uri).stm_power_cycle()
        time.sleep(4)

    print(f"Connecting to {args.uri}")
    cf.open_link(args.uri)

    deadline = time.time() + 15
    while not state["ready"] and not state["closed"] and time.time() < deadline:
        time.sleep(0.1)

    if not state["ready"]:
        print("Could not get a full connection.")
        cf.close_link()
        return

    print_controls()
    print(f"Command rate: {args.command_hz:g} Hz")
    print("Arming at target height 0. Press r slowly to climb, l to land.")
    arm(cf, True)

    last_status = 0.0
    try:
        while True:
            while msvcrt.kbhit():
                key = msvcrt.getwch().lower()

                if key in {" ", "x"}:
                    print("EMERGENCY STOP")
                    stop_now(cf)
                    cf.close_link()
                    return
                if key == "?":
                    print_controls()
                elif key == "l":
                    state["landing"] = True
                    state["vx_target"] = 0.0
                    state["vy_target"] = 0.0
                    state["yawrate_target"] = 0.0
                    print("\nLanding")
                elif key == "r":
                    state["landing"] = False
                    state["height_target"] = clamp(state["height_target"] + HEIGHT_STEP_M, MIN_HEIGHT_M, MAX_HEIGHT_M)
                elif key == "f":
                    state["height_target"] = clamp(state["height_target"] - HEIGHT_STEP_M, MIN_HEIGHT_M, MAX_HEIGHT_M)
                elif key == "w":
                    state["vx_target"] = clamp(state["vx_target"] + VELOCITY_STEP_M_S, -MAX_VELOCITY_M_S, MAX_VELOCITY_M_S)
                elif key == "s":
                    state["vx_target"] = clamp(state["vx_target"] - VELOCITY_STEP_M_S, -MAX_VELOCITY_M_S, MAX_VELOCITY_M_S)
                elif key == "a":
                    state["vy_target"] = clamp(state["vy_target"] + VELOCITY_STEP_M_S, -MAX_VELOCITY_M_S, MAX_VELOCITY_M_S)
                elif key == "d":
                    state["vy_target"] = clamp(state["vy_target"] - VELOCITY_STEP_M_S, -MAX_VELOCITY_M_S, MAX_VELOCITY_M_S)
                elif key == "q":
                    state["yawrate_target"] = clamp(state["yawrate_target"] - YAW_STEP_DPS, -MAX_YAW_DPS, MAX_YAW_DPS)
                elif key == "e":
                    state["yawrate_target"] = clamp(state["yawrate_target"] + YAW_STEP_DPS, -MAX_YAW_DPS, MAX_YAW_DPS)
                elif key == "c":
                    state["vx_target"] = 0.0
                    state["vy_target"] = 0.0
                    state["yawrate_target"] = 0.0

            if state["landing"]:
                state["height_target"] = max(0.0, state["height_target"] - LAND_RATE_M_S * command_period_s)
                if state["height_target"] <= 0.001:
                    print("\nLanded: stopping and disarming")
                    stop_now(cf)
                    cf.close_link()
                    return

            max_delta = RAMP_PER_SECOND * command_period_s
            state["vx"] = approach(state["vx"], state["vx_target"], max_delta)
            state["vy"] = approach(state["vy"], state["vy_target"], max_delta)
            state["yawrate"] = approach(state["yawrate"], state["yawrate_target"], max_delta * 200.0)
            state["height"] = approach(state["height"], state["height_target"], max_delta * 0.20)

            cf.commander.send_hover_setpoint(
                state["vx"],
                state["vy"],
                state["yawrate"],
                state["height"],
            )

            now = time.time()
            if now - last_status > 0.5:
                print(
                    f"vx={state['vx']:5.2f} vy={state['vy']:5.2f} "
                    f"yawrate={state['yawrate']:5.1f} z={state['height']:4.2f} "
                    f"target_z={state['height_target']:4.2f}",
                    end="\r",
                )
                last_status = now

            time.sleep(command_period_s)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stopping")
    finally:
        stop_now(cf)
        cf.close_link()
        time.sleep(0.5)


if __name__ == "__main__":
    main()
