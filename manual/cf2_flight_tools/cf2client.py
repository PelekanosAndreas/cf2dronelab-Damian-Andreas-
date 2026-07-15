"""
This module provides a client for controlling the Crazyflie 2.0 drone using the Crazyflie Python library. 
It allows you to connect to the drone, send commands, and receive telemetry data.

All this can be found in the crazyflie python lubrary documentation and examples, but I have added comments to explain the purpose of each import and the initialization steps.


CF2Client is currently a default-PID Crazyflie client with configurable telemetry logging.
It has early optional marker-deck configuration, but mocap is not actually integrated yet.
Custom controller support is not active.

"""

# Standard library imports
import csv
import datetime
import json
import logging
import threading
import time
from pathlib import Path
import numpy as np



# Crazyflie library imports
import cflib.crtp  # Initializes the Crazyflie radio/USB communication drivers.
from cflib.crazyflie import Crazyflie  # Main class for managing the drone connection.
from cflib.utils.power_switch import PowerSwitch # Utility for controlling the drone's power state via the radio.
from cflib.crazyflie.log import LogConfig  # Defines telemetry variables and logging rate.


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Initialize radio and the low-level drivers
cflib.crtp.init_drivers()


class CF2Client:
    """
    Crazyflie 2.x client with user-selectable telemetry logging.

    Logging modes:
        none:   Do not log telemetry.
        memory: Store telemetry in RAM. Use export_log(...) after flight.
        csv:    Write each telemetry packet directly to CSV.
        json:   Write each telemetry packet directly as JSON lines.
    """

    VALID_LOG_MODES = {"none", "memory", "csv", "json", "jsonl"}

    def __init__(
        self,
        uri,
        marker_deck_ids,
        log_variables=None,
        reboot_before_connect=True,
        log_mode="memory",
        log_hz=100,
        log_output_path=None,
        max_variables_per_config=5,
        flush_immediate=False,
        idle_thrust=3000,
        min_battery_voltage=3.5,
        battery_check_timeout=3.0,
        live_map=None,
        live_map_hz=20,
        motor_warmup_power=5000,
        motor_warmup_duration=5.5,
    ):
        """
        Initialize the CF2Client instance.

        Args:
            uri (str): Crazyflie radio URI.
            log_variables (list, optional): Crazyflie log variable names.
            reboot_before_connect (bool): Power-cycle before connecting.
            log_mode (str): "none", "memory", "csv", "json", or "jsonl".
            log_hz (float): Requested logging rate in Hz.
            log_output_path (str or Path, optional): Output for csv/json modes.
            max_variables_per_config (int): Variables per Crazyflie LogConfig.
            flush_immediate (bool): Flush file logs after each packet.
            idle_thrust (int, optional): Value for powerDist.idleThrust.
            min_battery_voltage (float, optional): Minimum allowed battery voltage.
            battery_check_timeout (float): Seconds to wait for battery telemetry.
            live_map (object, optional): Object with update_position(...) and set_target(...).
            live_map_hz (float): Maximum rate for live map position updates.
            motor_warmup_power (int): Direct motor power for preflight warmup.
            motor_warmup_duration (float): Seconds to hold warmup motor power.
        """

        # Address of Crazyflie
        self.uri = uri

        # 2 Things: Create the Crazyflie connection object and the cache stores downloaded firmware metadata, such as parameters and log so future connections can start faster.
        self.cf = Crazyflie(rw_cache="./__cfcache__")

        # User can list parameters to track. (Damian or Andreas... later we should create check to make sure parameters are valid naming since we downloaded metadata)
        if log_variables is None:
            self.log_variables = []
        elif isinstance(log_variables, list):
            self.log_variables = list(log_variables)
        else:
            raise ValueError("log_variables must be a list of log variable names or None.")
        
        # Stores the logging mode the user chose.
        self.log_mode = self._normalize_log_mode(log_mode)

        # 
        self.marker_deck_ids = marker_deck_ids
        self.live_map = live_map


        self.log_hz = self._validate_log_hz(log_hz)
        self.log_period_ms = max(1, int(round(1000 / self.log_hz)))
        self.log_output_path = Path(log_output_path) if log_output_path else None
        self.max_variables_per_config = self._validate_max_variables(max_variables_per_config)
        self.flush_immediate = flush_immediate
        self.idle_thrust = self._validate_idle_thrust(idle_thrust)
        self.min_battery_voltage = self._validate_min_battery_voltage(min_battery_voltage)
        self.battery_check_timeout = self._validate_positive_float(
            battery_check_timeout,
            "battery_check_timeout",
        )
        self.live_map_period_s = 1.0 / self._validate_positive_float(live_map_hz, "live_map_hz")
        self.motor_warmup_power = self._validate_motor_warmup_power(motor_warmup_power)
        self.motor_warmup_duration = self._validate_non_negative_float(
            motor_warmup_duration,
            "motor_warmup_duration",
        )

        # This is Connection Status Flag... Later this will be changed to true when drone is ready to takeoff affect pretty flight checklist
        self.is_fully_connected = False 

        self.logconfs = []
        self.log_records = []
        self.log_data = {}
        self.latest_log_values = {}
        self._log_file = None
        self._csv_writer = None
        self._preflight_started = False
        self.battery_before = None
        self.battery_after = None
        self.current_position = None
        self.current_target = None
        self.takeoff_position = None
        self.connection_error = None
        self._last_live_map_update_time = 0.0

        # Register callbacks with crazyflie-library and crazyflie class
        self.cf.connected.add_callback(self._connected)
        self.cf.fully_connected.add_callback(self._fully_connected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)
        self.cf.disconnected.add_callback(self._disconnected)

        # Reboot crazyflie before establishing a connection
        if reboot_before_connect:
            print(f"CF2Client: Rebooting drone at {uri}")
            try:
                PowerSwitch(uri).stm_power_cycle()
                time.sleep(4) # Damian or Andreas... we might want to automate this later.
            except Exception as exc:
                print(f"CF2Client: Reboot failed, continuing without reboot: {exc}")

        # Start the connection process
        print(f"CF2Client: Starting to connect to {uri}")
        self.cf.open_link(uri)


    # Defining Callbacks that have already been registered above
    def _connected(self, uri):
        print(f"CF2Client: Link established to {uri}.")

    def _connection_failed(self, uri, msg):
        self.connection_error = msg
        print(f"CF2Client: Connection failed to {uri}: {msg}")

    def _connection_lost(self, uri, msg):
        print(f"CF2Client: Connection lost from {uri}: {msg}")

    def _fully_connected(self, uri):
        print(f"CF2Client: Fully connected to {uri}. Starting preflight checks.")
        if self._preflight_started:
            return
        self._preflight_started = True
        threading.Thread(
            target=self._run_preflight_after_connect,
            args=(uri,),
            daemon=True,
        ).start()

    def _run_preflight_after_connect(self, uri):
        if self.start_preflight_checks():

            #
            self.start_logging() 
            # This the flag that tells the flight.py script execute.
            self.is_fully_connected = True 
            print("CF2Client: Ready.")
        else:
            print(f"CF2Client: Preflight checks failed for {uri}")

    def _disconnected(self, uri):
        self.is_fully_connected = False
        print(f"CF2Client: Disconnected from {uri}")

    # Similar to how pilot checks certain systems that are critcal system before even warming up engines before takeoff

    def start_preflight_checks(self):
        """
        Run preflight checks after the Crazyflie link is established.

        Returns:
            bool: True if all checks pass, False otherwise.
        """

        print("CF2Client: Starting preflight checks.")


        ############ Check 1: Look at Log Variables
        """
        This checks user manifest to see if there are any variables beginning logged or not.
        If the user chose no logging OR there are no variables to log... then warn them with message.
        """
        if self.log_mode == "none" or not self.log_variables:
            print("CF2Client: Telemetry logging disabled because log_mode = None or list or list of log_varables is Null")
            return
    

        ############ Check 2: Inititalize Crazy Decks
        """
        Not all of the deck are included so add more if needed.....
        """

        # Active Marker Deck
        """
        This check configures the Crazyflie’s active marker deck for the Qualisys motion capture system.
        """

        if self.marker_deck_ids is not None:
            print(f'CrazyflieClient: Using active marker deck with IDs {self.marker_deck_ids}')

            # Set the marker mode (3: qualisys)
            self.cf.param.set_value('activeMarker.mode', 3)

            # Set the marker IDs
            self.cf.param.set_value('activeMarker.front', self.marker_deck_ids[0])
            self.cf.param.set_value('activeMarker.right', self.marker_deck_ids[1])
            self.cf.param.set_value('activeMarker.back', self.marker_deck_ids[2])
            self.cf.param.set_value('activeMarker.left', self.marker_deck_ids[3])

        # Flow Deck
        

        ############ Check 3: Reset Crazyflie’s Default State Estimator
        """
        This resets the Crazyflie’s default state estimator, which here is the Kalman filter. 
        So this block is basically saying: Start fresh. Forget the old position/orientation estimate and reinitialize.
        """

        # Sets the reset flag to 1, telling the firmware: “reset the Kalman estimator now.”
        self.cf.param.set_value('kalman.resetEstimation', 1)
        time.sleep(0.1)

        # Clears the reset flag back to 0.
        self.cf.param.set_value('kalman.resetEstimation', 0)



        ############ Check 4: Set the Controller 
        """
        Later we can use additional controllers but for now we will keep this as Cascaded PID controller
        """

        self.cf.param.set_value('stabilizer.controller', 1)



        ############ Check 5: Set Idle Thrust
        """
        Set the motor idle thrust parameter before flight.
        """
        if self.idle_thrust is not None:
            self.cf.param.set_value('powerDist.idleThrust', str(self.idle_thrust))



        ############ Check 6: Setup Storage and set log recieving  
        """
        A LogConfig tells the Crazyflie: Send me these variables every N milliseconds and this time is set by user.
        """
    
        # Initate Empty Storag Container for LogConfig Objects
        self.logconfs = []
    
        """
        This deserves some explanation. Right now with check 2 in preflight list the client will now prepare storage need what crazyflie class class LogConfig.
        A LogConfig is a Crazyflie logging setup that says: log these variables at this rate. For example, later the code creates one like:
        LogConfig(name="LogConf0", period_in_ms=10) and stores it in the list: self.logconfs.append(current_logconf).So after setup, self.logconfs might 
        look conceptually like:

        [
            LogConf0,
            LogConf1
        ]
        Why a list? Because if you want to log many variables ('stateEstimate.x',..., etc.), the code may need multiple LogConfig objects. 
        Crazyflie log packets have size limits (interaction with radio commonly can be seen when flashing the drone), so the variables may be split across several configs.
    
        """

        # 
        current_logconf = None
        num_variables = 0

       # Create a new LogConfig if: 1. We do not have one yet OR 2. The current one already has the maximum allowed number of variables
        for variable in self.log_variables:
            if current_logconf is None or num_variables >= self.max_variables_per_config:
                current_logconf = LogConfig(
                    name=f"LogConf{len(self.logconfs)}",
                    period_in_ms=self.log_period_ms,
                )
                self.logconfs.append(current_logconf)
                num_variables = 0

            current_logconf.add_variable(variable)
            num_variables += 1


        ############ Check 7: Battery
        if not self.check_battery():
            return False

        ############ Check 8: Arm Done
        time.sleep(0.5)
        print("Arming...")
        if hasattr(self.cf, "supervisor"):
            self.cf.supervisor.send_arming_request(True)
        else:
            self.cf.platform.send_arming_request(True)
        time.sleep(0.5)
        print("Armed")


        # Preflight Checks Complete

        return True

    def check_battery(self):
        """
        Read battery telemetry and fail preflight if the battery is too low.

        Returns:
            bool: True if battery telemetry is acceptable, False otherwise.
        """

        if self.min_battery_voltage is None:
            print("CF2Client: Battery check disabled.")
            return True

        print("CF2Client: Checking battery.")
        battery = self._read_battery_once()
        if battery is None:
            print("CF2Client: Battery check failed: no battery telemetry received.")
            return False

        self.battery_before = self._normalize_battery_snapshot(battery)
        voltage = self.battery_before.get("voltage_v")
        parts = []

        if voltage is not None:
            parts.append(f"{voltage:.2f} V")
        print(f"Battery before flight: {' / '.join(parts) if parts else 'telemetry received'}.")

        if self.min_battery_voltage is not None:
            if voltage is None:
                print("CF2Client: Battery check failed: pm.vbat was not available.")
                return False
            if voltage <= self.min_battery_voltage:
                print(
                    "CF2Client: Battery check failed: "
                    f"{voltage:.2f} V <= {self.min_battery_voltage:.2f} V."
                )
                return False

        return True

    def read_battery_snapshot(self):
        """
        Read battery telemetry and return a normalized snapshot for output.
        """

        battery = self._read_battery_once()
        if battery is None:
            battery = {
                variable: self.latest_log_values[variable]
                for variable in ("pm.vbat", "pm.vbatMV")
                if variable in self.latest_log_values
            }

        return self._normalize_battery_snapshot(battery) if battery else None

    def _normalize_battery_snapshot(self, battery):
        voltage = battery.get("pm.vbat")
        voltage_mv = battery.get("pm.vbatMV")
        if voltage is None and voltage_mv is not None:
            voltage = voltage_mv / 1000.0

        return {
            "host_time_s": time.time(),
            "voltage_v": voltage,
            "voltage_mv": voltage_mv,
            "raw": dict(battery),
        }

    def _read_battery_once(self):
        deadline = time.time() + self.battery_check_timeout
        attempt = 0
        while time.time() < deadline:
            battery = {}
            received = threading.Event()
            logconf = LogConfig(name=f"BatteryCheck{attempt}", period_in_ms=100)
            added_variables = []

            for variable in ("pm.vbat", "pm.vbatMV"):
                try:
                    logconf.add_variable(variable)
                    added_variables.append(variable)
                except KeyError:
                    pass

            if not added_variables:
                print("CF2Client: Battery check could not find pm battery log variables.")
                return None

            def battery_data(_timestamp, data, _logconf):
                battery.update(data)
                if any(variable in battery for variable in added_variables):
                    received.set()

            def battery_error(_logconf, msg):
                print(f"CF2Client: Battery log error: {msg}")
                received.set()

            try:
                logconf.data_received_cb.add_callback(battery_data)
                logconf.error_cb.add_callback(battery_error)
                self.cf.log.add_config(logconf)
                logconf.start()
                received.wait(min(1.0, max(0.1, deadline - time.time())))
            except (KeyError, AttributeError) as exc:
                print(f"CF2Client: Battery check could not start: {exc}")
                return None
            finally:
                try:
                    logconf.stop()
                except Exception:
                    pass

            if battery:
                return battery

            attempt += 1
            time.sleep(0.1)

        return None
    




    ######################## Logging Data Methods #############################
    def start_logging(self):
        """Create LogConfig objects and start telemetry logging."""

        # Start Memory log
        self._prepare_memory_log()

        
        if self.log_mode in {"csv", "jsonl"}:
            self._open_immediate_log_file()

        for logconf in self.logconfs:
            try:
                self.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self._log_data)
                logconf.error_cb.add_callback(self._log_error)
                logconf.start()
            except KeyError as exc:
                print(f"CF2Client: Could not start {logconf.name}: {exc}")
            except AttributeError:
                print(f"CF2Client: Could not start {logconf.name}: bad configuration")

        print(
            f"CF2Client: Logging {len(self.log_variables)} variables "
            f"at {self.log_hz:g} Hz in {self.log_mode} mode."
        )

    
    def stop_logging(self):
        """Stop telemetry logging and close any immediate output file."""

        for logconf in self.logconfs:
            try:
                logconf.stop()
            except Exception:
                pass

        self.logconfs = []

        if self._log_file:
            self._log_file.flush()
            self._log_file.close()
            self._log_file = None
            self._csv_writer = None

    def export_log(self, output_path, output_format=None):
        """
        Export memory-backed logs after flight.

        Args:
            output_path (str or Path): Destination file path.
            output_format (str, optional): "csv" or "json". If omitted, the
                suffix of output_path is used.
        """

        output_path = Path(output_path)
        output_format = self._infer_export_format(output_path, output_format)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        if output_format == "csv":
            self._export_memory_csv(output_path)
        elif output_format == "json":
            self._export_memory_json(output_path)
        else:
            raise ValueError('output_format must be "csv" or "json".')

        print(f"CF2Client: Exported memory log to {output_path}")

    def _log_data(self, timestamp, data, logconf):
        host_time_s = time.time()
        timestamp_ms = timestamp / 1e3
        values = dict(data)

        record = {
            "timestamp_ms": timestamp_ms,
            "host_time_s": host_time_s,
            "logconf": logconf.name,
            "values": values,
        }

        self.latest_log_values.update(values)
        self._update_live_map(host_time_s)

        if self.log_mode == "memory":
            self.log_records.append(record)

            for variable, value in values.items():
                if variable not in self.log_data:
                    self.log_data[variable] = {"time_ms": [], "host_time_s": [], "data": []}
                self.log_data[variable]["time_ms"].append(timestamp_ms)
                self.log_data[variable]["host_time_s"].append(host_time_s)
                self.log_data[variable]["data"].append(value)

        if self.log_mode == "csv":
            row = {
                "timestamp_ms": timestamp_ms,
                "host_time_s": host_time_s,
                "logconf": logconf.name,
            }
            row.update(values)
            self._csv_writer.writerow(row)
            self._flush_if_needed()
        elif self.log_mode == "jsonl":
            self._log_file.write(json.dumps(record) + "\n")
            self._flush_if_needed()

    def _log_error(self, logconf, msg):
        print(f"CF2Client: Error when logging {logconf.name}: {msg}")

    def _prepare_memory_log(self):
        self.log_records = []
        self.latest_log_values = {}

        if self.log_mode == "memory":
            self.log_data = {
                variable: {"time_ms": [], "host_time_s": [], "data": []}
                for variable in self.log_variables
            }
        else:
            self.log_data = {}

    def _open_immediate_log_file(self):
        if self.log_output_path is None:
            self.log_output_path = self._default_log_output_path()

        self.log_output_path.parent.mkdir(parents=True, exist_ok=True)

        if self.log_mode == "csv":
            self._log_file = open(self.log_output_path, "w", newline="")
            fieldnames = ["timestamp_ms", "host_time_s", "logconf"] + self.log_variables
            self._csv_writer = csv.DictWriter(self._log_file, fieldnames=fieldnames)
            self._csv_writer.writeheader()
        elif self.log_mode == "jsonl":
            self._log_file = open(self.log_output_path, "w")

        print(f"CF2Client: Immediate log output is {self.log_output_path}")

    def _export_memory_csv(self, output_path):
        fieldnames = ["timestamp_ms", "host_time_s", "logconf"] + self.log_variables

        with open(output_path, "w", newline="") as output_file:
            writer = csv.DictWriter(output_file, fieldnames=fieldnames)
            writer.writeheader()

            for record in self.log_records:
                row = {
                    "timestamp_ms": record["timestamp_ms"],
                    "host_time_s": record["host_time_s"],
                    "logconf": record["logconf"],
                }
                row.update(record["values"])
                writer.writerow(row)

    def _export_memory_json(self, output_path):
        payload = {
            "log_hz": self.log_hz,
            "log_period_ms": self.log_period_ms,
            "variables": self.log_variables,
            "records": self.log_records,
            "by_variable": self.log_data,
        }

        with open(output_path, "w") as output_file:
            json.dump(payload, output_file, indent=2)

    def _flush_if_needed(self):
        if self.flush_immediate and self._log_file:
            self._log_file.flush()

    def _default_log_output_path(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        suffix = "csv" if self.log_mode == "csv" else "jsonl"
        return Path("outputs") / f"cf2_log_{timestamp}.{suffix}"

    def _normalize_log_mode(self, log_mode):
        normalized = str(log_mode).lower()
        if normalized == "json":
            normalized = "jsonl"

        if normalized not in self.VALID_LOG_MODES:
            raise ValueError(
                'log_mode must be one of "none", "memory", "csv", "json", or "jsonl".'
            )

        return normalized

    def _validate_log_hz(self, log_hz):
        log_hz = float(log_hz)
        if log_hz <= 0:
            raise ValueError("log_hz must be greater than 0.")
        return log_hz

    def _validate_max_variables(self, max_variables_per_config):
        max_variables_per_config = int(max_variables_per_config)
        if max_variables_per_config <= 0:
            raise ValueError("max_variables_per_config must be greater than 0.")
        return max_variables_per_config

    def _validate_idle_thrust(self, idle_thrust):
        if idle_thrust is None:
            return None

        idle_thrust = int(idle_thrust)
        if idle_thrust < 0 or idle_thrust > 8000:
            raise ValueError("for safety idle_thrust must be between 0 and 8000.")

        return idle_thrust

    def _validate_min_battery_voltage(self, min_battery_voltage):
        if min_battery_voltage is None:
            return None

        min_battery_voltage = float(min_battery_voltage)
        if min_battery_voltage < 3.0 or min_battery_voltage > 4.3:
            raise ValueError("min_battery_voltage must be between 3.0 and 4.3 volts.")

        return min_battery_voltage

    def _validate_positive_float(self, value, name):
        value = float(value)
        if value <= 0:
            raise ValueError(f"{name} must be greater than 0.")

        return value

    def _validate_non_negative_float(self, value, name):
        value = float(value)
        if value < 0:
            raise ValueError(f"{name} must be greater than or equal to 0.")

        return value

    def _validate_motor_warmup_power(self, motor_warmup_power):
        motor_warmup_power = int(motor_warmup_power)
        if motor_warmup_power < 0 or motor_warmup_power > 20000:
            raise ValueError("motor_warmup_power must be between 0 and 20000.")

        return motor_warmup_power

    def _infer_export_format(self, output_path, output_format):
        if output_format is not None:
            return str(output_format).lower()

        suffix = output_path.suffix.lower()
        if suffix == ".csv":
            return "csv"
        if suffix in {".json", ".jsonl"}:
            return "json"

        raise ValueError('Could not infer output format. Use ".csv" or ".json".')
    
    def disconnect(self):
        """Stop logging and close the Crazyflie link."""

        self.stop_logging()
        self.cf.close_link()


    ######################## Basic Flight Commands #############################

    def move(self, x, y, z, yaw=0.0, duration=2.0, command_dt=0.1):
        """
        Hold one position setpoint for a fixed duration.

        This sends the same desired x/y/z/yaw command repeatedly. It does not
        check whether the drone has reached the point.
        """

        print(f'CF2Client: Move to {x}, {y}, {z} with yaw {yaw} degrees for {duration} seconds')
        self._set_live_map_target(x, y, z)
        command_dt = self._validate_positive_float(command_dt, "command_dt")
        start_time = time.perf_counter()
        end_time = start_time + duration
        next_command_time = start_time
        command_count = 0

        while next_command_time < end_time:
            self._sleep_until_perf(next_command_time)
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            command_count += 1
            next_command_time += command_dt

        self._sleep_until_perf(end_time)

        return command_count


    def move_smooth(self, p_inW_1, p_inW_2, yaw, v):
        print(f'Move smoothly from {p_inW_1} to {p_inW_2} with yaw {yaw} degrees at {v} meters / second')

        # Make sure p_inW_1 and p_inW_2 are numpy arrays
        p_inW_1 = np.array(p_inW_1) + [
            0.0,
            0.0,
            0.0
        ]
        p_inW_2 = np.array(p_inW_2) + [
            0.0,
            0.0,
            0.0
        ]
        
        # Compute distance from p_inW_1 to p_inW_2
        d = np.linalg.norm(p_inW_2-p_inW_1)
        
        # Compute time it takes to move from p_inW_1 to p_inW_2 at desired speed
        dt = d/v
        
        # Get start time
        start_time = time.time()

        # Repeat until the current time is dt seconds later than the start time
        while True:
            # Get the current time
            t = time.time()
            
            # Compute what fraction of the distance from p_inW_1 to p_inW_2
            # should have been travelled by the current time
            if t < start_time:
                s = 0
            elif start_time<=t<=start_time+dt:
                s = (t-start_time)/dt
            else:
                s = 1
            
            # Compute where the drone should be at the current time, in the
            # coordinates of the world frame
            p_inW_des = (1-s)*p_inW_1 + s*p_inW_2
            
            # Send the desired position (and yaw angle) to the drone
            self.cf.commander.send_position_setpoint(p_inW_des[0], p_inW_des[1], p_inW_des[2], yaw)

            # Stop if the move is complete (i.e., if the desired position is at p_inW_2)
            # otherwise pause for 0.1 seconds before sending another desired position
            if s >= 1:
                return
            else:
                time.sleep(0.1)

    def motor_idle_test(self, power=None, duration=None, ramp_time=1.5, ramp_down=True):
        """
        Spin all motors at low direct power before the first flight command.

        This is only a motor warmup. It does not control position or altitude.
        """

        power = self.motor_warmup_power if power is None else self._validate_motor_warmup_power(power)
        duration = self.motor_warmup_duration if duration is None else self._validate_non_negative_float(
            duration,
            "duration",
        )

        if power <= 0 or duration <= 0:
            print("CF2Client: Motor warmup disabled.")
            return

        print(f"CF2Client: Motor warmup at {power} for {duration:.1f} seconds.")
        step_dt = 0.05
        ramp_steps = max(1, int(ramp_time / step_dt))

        try:
            for step in range(1, ramp_steps + 1):
                step_power = int(power * step / ramp_steps)
                self._set_direct_motor_power(step_power)
                time.sleep(step_dt)

            end_time = time.time() + duration
            while time.time() < end_time:
                self._set_direct_motor_power(power)
                time.sleep(step_dt)

            if ramp_down:
                for step in range(ramp_steps - 1, -1, -1):
                    step_power = int(power * step / ramp_steps)
                    self._set_direct_motor_power(step_power)
                    time.sleep(step_dt)
        except (KeyError, AttributeError) as exc:
            print(f"CF2Client: Motor warmup skipped: {exc}")
        finally:
            try:
                self._set_direct_motor_power(0)
                self.cf.param.set_value("motorPowerSet.enable", "0")
            except (KeyError, AttributeError):
                pass

    def _set_direct_motor_power(self, power):
        power = int(power)
        self.cf.param.set_value("motorPowerSet.enable", "1")
        self.cf.param.set_value("motorPowerSet.m1", str(power))
        self.cf.param.set_value("motorPowerSet.m2", str(power))
        self.cf.param.set_value("motorPowerSet.m3", str(power))
        self.cf.param.set_value("motorPowerSet.m4", str(power))

    def move_smooth_to(self, x, y, z, yaw=0.0, duration=2.0, command_dt=0.1):
        """
        Move from the latest known target/position to a new target over duration.
        """

        start_position = self._smooth_move_start_position()
        end_position = np.array([float(x), float(y), float(z)])
        print(
            "CF2Client: Smooth move to "
            f"{x}, {y}, {z} with yaw {yaw} degrees for {duration} seconds"
        )
        self._set_live_map_target(x, y, z)

        command_dt = self._validate_positive_float(command_dt, "command_dt")
        start_time = time.perf_counter()
        end_time = start_time + duration
        next_command_time = start_time
        command_count = 0

        while next_command_time < end_time:
            self._sleep_until_perf(next_command_time)
            elapsed = min(duration, next_command_time - start_time)
            fraction = min(1.0, max(0.0, elapsed / duration)) if duration > 0 else 1.0
            fraction = fraction * fraction * (3.0 - 2.0 * fraction)
            desired = start_position + (end_position - start_position) * fraction
            self.cf.commander.send_position_setpoint(
                desired[0],
                desired[1],
                desired[2],
                yaw,
            )
            command_count += 1
            next_command_time += command_dt

        self._sleep_until_perf(end_time)
        self.current_target = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "host_time_s": time.time(),
        }
        return command_count

    def _sleep_until_perf(self, target_time):
        delay = target_time - time.perf_counter()
        if delay > 0:
            time.sleep(delay)

    def _smooth_move_start_position(self):
        if self.current_target is not None:
            return np.array([
                self.current_target["x"],
                self.current_target["y"],
                self.current_target["z"],
            ])

        return np.array([0.0, 0.0, 0.0])

    def _set_live_map_target(self, x, y, z):
        self.current_target = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "host_time_s": time.time(),
        }

        if self.live_map is not None:
            self.live_map.set_target(x, y, z)

    def _update_live_map(self, host_time_s):
        x = self._optional_latest_float("stateEstimate.x")
        y = self._optional_latest_float("stateEstimate.y")
        z = self._optional_latest_float("stateEstimate.z")
        if x is None or y is None or z is None:
            return

        self.current_position = {
            "x": x,
            "y": y,
            "z": z,
            "host_time_s": host_time_s,
        }

        if self.takeoff_position is None:
            self.takeoff_position = dict(self.current_position)
            if self.live_map is not None:
                self.live_map.set_home(x, y, z)

        if self.live_map is None:
            return

        if host_time_s - self._last_live_map_update_time < self.live_map_period_s:
            return

        self._last_live_map_update_time = host_time_s
        self.live_map.update_position(
            x,
            y,
            z,
            host_time_s=host_time_s,
            vx=self._optional_latest_float("stateEstimate.vx"),
            vy=self._optional_latest_float("stateEstimate.vy"),
            vz=self._optional_latest_float("stateEstimate.vz"),
        )

    def _optional_latest_float(self, variable):
        value = self.latest_log_values.get(variable)
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def estimated_home_error(self):
        if self.takeoff_position is None or self.current_position is None:
            return None

        dx = self.current_position["x"] - self.takeoff_position["x"]
        dy = self.current_position["y"] - self.takeoff_position["y"]
        dz = self.current_position["z"] - self.takeoff_position["z"]
        return {
            "dx_m": dx,
            "dy_m": dy,
            "dz_m": dz,
            "xy_m": (dx ** 2 + dy ** 2) ** 0.5,
            "xyz_m": (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5,
        }


    def move_csv_waypoints(self, csv_path, default_yaw=0.0, default_duration=2.0):
        """
        Read waypoints from a CSV file and fly through them.

        Required CSV columns:
            x, y, z

        Optional CSV columns:
            yaw, duration
        """

        csv_path = Path(csv_path)
        print(f'CF2Client: Flying CSV waypoints from {csv_path}')

        with open(csv_path, newline='') as csv_file:
            reader = csv.DictReader(csv_file)

            for row_number, row in enumerate(reader, start=2):
                try:
                    x = float(row['x'])
                    y = float(row['y'])
                    z = float(row['z'])
                    yaw = float(row.get('yaw') or default_yaw)
                    duration = float(row.get('duration') or default_duration)
                except KeyError as exc:
                    raise ValueError(f'Missing required CSV column: {exc}') from exc
                except ValueError as exc:
                    raise ValueError(f'Bad waypoint value on CSV row {row_number}: {row}') from exc

                self.move(x, y, z, yaw=yaw, duration=duration)

    def stop(self, duration=1.0):
        print(f'CF2Client: Stop for {duration} seconds')
        self.cf.commander.send_stop_setpoint()
        self.cf.commander.send_notify_setpoint_stop()
        start_time = time.time()
        while time.time() - start_time < duration:
            time.sleep(0.1)

    
    
