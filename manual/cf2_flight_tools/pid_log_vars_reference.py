#
"""
Reference list for Crazyflie default PID-controller log variables.

This file is meant to help choose variables for a Crazyradio log config.
It does not connect to the Crazyflie by itself; it prints grouped log
variables with definitions and recommended use.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class LogVar:
    name: str
    definition: str
    use_when: str
    useful: bool = False


LOG_VARS = [
    # State estimates
    LogVar("stateEstimate.x", "Estimated world-frame x position [m].", "Compare actual x position to ctrltarget.x.", True),
    LogVar("stateEstimate.y", "Estimated world-frame y position [m].", "Compare actual y position to ctrltarget.y.", True),
    LogVar("stateEstimate.z", "Estimated world-frame z position [m].", "Compare actual height to ctrltarget.z.", True),
    LogVar("stateEstimate.vx", "Estimated world-frame x velocity [m/s].", "Debug velocity tracking or motion drift."),
    LogVar("stateEstimate.vy", "Estimated world-frame y velocity [m/s].", "Debug velocity tracking or motion drift."),
    LogVar("stateEstimate.vz", "Estimated world-frame z velocity [m/s].", "Debug altitude/vertical velocity behavior.", True),
    LogVar("stateEstimate.ax", "Estimated x acceleration [G].", "Inspect estimator acceleration output."),
    LogVar("stateEstimate.ay", "Estimated y acceleration [G].", "Inspect estimator acceleration output."),
    LogVar("stateEstimate.az", "Estimated z acceleration [G], gravity removed by estimator convention.", "Debug vertical motion."),
    LogVar("stateEstimate.roll", "Estimated roll angle [deg].", "Check attitude response and stability.", True),
    LogVar("stateEstimate.pitch", "Estimated pitch angle [deg].", "Check attitude response and stability.", True),
    LogVar("stateEstimate.yaw", "Estimated yaw angle [deg].", "Check heading/yaw tracking.", True),
    LogVar("stateEstimate.qx", "Estimated attitude quaternion x component.", "Use for quaternion-based attitude analysis."),
    LogVar("stateEstimate.qy", "Estimated attitude quaternion y component.", "Use for quaternion-based attitude analysis."),
    LogVar("stateEstimate.qz", "Estimated attitude quaternion z component.", "Use for quaternion-based attitude analysis."),
    LogVar("stateEstimate.qw", "Estimated attitude quaternion w component.", "Use for quaternion-based attitude analysis."),

    # Desired setpoint / commander target
    LogVar("ctrltarget.x", "Commanded x position [m].", "Compare with stateEstimate.x.", True),
    LogVar("ctrltarget.y", "Commanded y position [m].", "Compare with stateEstimate.y.", True),
    LogVar("ctrltarget.z", "Commanded z position [m].", "Compare with stateEstimate.z.", True),
    LogVar("ctrltarget.vx", "Commanded x velocity [m/s].", "Use when sending velocity setpoints."),
    LogVar("ctrltarget.vy", "Commanded y velocity [m/s].", "Use when sending velocity setpoints."),
    LogVar("ctrltarget.vz", "Commanded z velocity [m/s].", "Use when sending velocity setpoints."),
    LogVar("ctrltarget.ax", "Commanded x acceleration [m/s^2].", "Use with trajectory/high-level setpoints."),
    LogVar("ctrltarget.ay", "Commanded y acceleration [m/s^2].", "Use with trajectory/high-level setpoints."),
    LogVar("ctrltarget.az", "Commanded z acceleration [m/s^2].", "Use with trajectory/high-level setpoints."),
    LogVar("ctrltarget.roll", "Commanded roll angle [deg].", "Use when flying attitude/manual mode."),
    LogVar("ctrltarget.pitch", "Commanded pitch angle [deg].", "Use when flying attitude/manual mode."),
    LogVar("ctrltarget.yaw", "Commanded yaw rate [deg/s] in this firmware log group.", "Use to inspect yaw-rate commands.", True),

    # Main PID controller outputs / targets
    LogVar("controller.cmd_thrust", "Final PID thrust command copied for logging.", "See commanded thrust before motor mixing.", True),
    LogVar("controller.cmd_roll", "Final PID roll command copied for logging.", "Debug roll control output.", True),
    LogVar("controller.cmd_pitch", "Final PID pitch command copied for logging.", "Debug pitch control output.", True),
    LogVar("controller.cmd_yaw", "Final PID yaw command copied for logging.", "Debug yaw control output.", True),
    LogVar("controller.r_roll", "Measured roll rate from gyro [rad/s].", "Compare gyro roll rate to controller.rollRate."),
    LogVar("controller.r_pitch", "Measured pitch rate from gyro [rad/s].", "Compare gyro pitch rate to controller.pitchRate."),
    LogVar("controller.r_yaw", "Measured yaw rate from gyro [rad/s].", "Compare gyro yaw rate to controller.yawRate."),
    LogVar("controller.accelz", "Measured z acceleration from accelerometer [G].", "Debug vertical response and thrust behavior."),
    LogVar("controller.actuatorThrust", "Thrust output from position/velocity controller before final assignment.", "Debug altitude PID output."),
    LogVar("controller.roll", "Desired roll angle from PID position/attitude path [deg].", "Inspect requested attitude.", True),
    LogVar("controller.pitch", "Desired pitch angle from PID position/attitude path [deg].", "Inspect requested attitude.", True),
    LogVar("controller.yaw", "Desired yaw angle maintained by PID controller [deg].", "Inspect yaw target.", True),
    LogVar("controller.rollRate", "Desired roll rate [deg/s].", "Debug attitude-to-rate loop."),
    LogVar("controller.pitchRate", "Desired pitch rate [deg/s].", "Debug attitude-to-rate loop."),
    LogVar("controller.yawRate", "Desired yaw rate [deg/s].", "Debug attitude-to-rate loop."),
    LogVar("controller.ctr_yaw", "Raw int16 yaw control output from stabilizer control struct.", "Low-level yaw output debugging."),

    # PID position controller internals
    LogVar("posCtl.targetX", "PID internal x-position target in yaw-aligned body frame [m].", "Debug position loop in x.", True),
    LogVar("posCtl.targetY", "PID internal y-position target in yaw-aligned body frame [m].", "Debug position loop in y.", True),
    LogVar("posCtl.targetZ", "PID internal z-position target in world frame [m].", "Debug altitude position loop.", True),
    LogVar("posCtl.targetVX", "PID internal x-velocity target in yaw-aligned body frame [m/s].", "Debug position-to-velocity command.", True),
    LogVar("posCtl.targetVY", "PID internal y-velocity target in yaw-aligned body frame [m/s].", "Debug position-to-velocity command.", True),
    LogVar("posCtl.targetVZ", "PID internal z-velocity target [m/s].", "Debug altitude velocity command.", True),
    LogVar("posCtl.bodyX", "Estimated x position rotated into yaw-aligned body frame [m].", "Compare to posCtl.targetX."),
    LogVar("posCtl.bodyY", "Estimated y position rotated into yaw-aligned body frame [m].", "Compare to posCtl.targetY."),
    LogVar("posCtl.bodyVX", "Estimated x velocity rotated into yaw-aligned body frame [m/s].", "Compare to posCtl.targetVX."),
    LogVar("posCtl.bodyVY", "Estimated y velocity rotated into yaw-aligned body frame [m/s].", "Compare to posCtl.targetVY."),

    # Position PID terms
    LogVar("posCtl.Xp", "X position PID proportional term.", "Tune or inspect x position loop."),
    LogVar("posCtl.Xi", "X position PID integral term.", "Check x integrator buildup."),
    LogVar("posCtl.Xd", "X position PID derivative term.", "Check x derivative response/noise."),
    LogVar("posCtl.Xff", "X position PID feed-forward term.", "Check x feed-forward contribution."),
    LogVar("posCtl.Yp", "Y position PID proportional term.", "Tune or inspect y position loop."),
    LogVar("posCtl.Yi", "Y position PID integral term.", "Check y integrator buildup."),
    LogVar("posCtl.Yd", "Y position PID derivative term.", "Check y derivative response/noise."),
    LogVar("posCtl.Yff", "Y position PID feed-forward term.", "Check y feed-forward contribution."),
    LogVar("posCtl.Zp", "Z position PID proportional term.", "Tune or inspect altitude position loop."),
    LogVar("posCtl.Zi", "Z position PID integral term.", "Check altitude integrator buildup."),
    LogVar("posCtl.Zd", "Z position PID derivative term.", "Check altitude derivative response/noise."),
    LogVar("posCtl.Zff", "Z position PID feed-forward term.", "Check altitude feed-forward contribution."),

    # Velocity PID terms
    LogVar("posCtl.VXp", "X velocity PID proportional term.", "Tune or inspect x velocity loop."),
    LogVar("posCtl.VXi", "X velocity PID integral term.", "Check x velocity integrator buildup."),
    LogVar("posCtl.VXd", "X velocity PID derivative term.", "Check x velocity derivative response/noise."),
    LogVar("posCtl.VXff", "X velocity PID feed-forward term.", "Check x velocity feed-forward contribution."),
    LogVar("posCtl.VYp", "Y velocity PID proportional term.", "Tune or inspect y velocity loop."),
    LogVar("posCtl.VYi", "Y velocity PID integral term.", "Check y velocity integrator buildup."),
    LogVar("posCtl.VYd", "Y velocity PID derivative term.", "Check y velocity derivative response/noise."),
    LogVar("posCtl.VYff", "Y velocity PID feed-forward term.", "Check y velocity feed-forward contribution."),
    LogVar("posCtl.VZp", "Z velocity PID proportional term.", "Tune or inspect z velocity loop."),
    LogVar("posCtl.VZi", "Z velocity PID integral term.", "Check z velocity integrator buildup."),
    LogVar("posCtl.VZd", "Z velocity PID derivative term.", "Check z velocity derivative response/noise."),
    LogVar("posCtl.VZff", "Z velocity PID feed-forward term.", "Check z velocity feed-forward contribution."),

    # Attitude PID terms
    LogVar("pid_attitude.roll_outP", "Roll attitude PID proportional term.", "Tune or inspect roll angle loop."),
    LogVar("pid_attitude.roll_outI", "Roll attitude PID integral term.", "Check roll angle integrator buildup."),
    LogVar("pid_attitude.roll_outD", "Roll attitude PID derivative term.", "Check roll angle derivative/noise."),
    LogVar("pid_attitude.roll_outFF", "Roll attitude PID feed-forward term.", "Check roll angle feed-forward."),
    LogVar("pid_attitude.pitch_outP", "Pitch attitude PID proportional term.", "Tune or inspect pitch angle loop."),
    LogVar("pid_attitude.pitch_outI", "Pitch attitude PID integral term.", "Check pitch angle integrator buildup."),
    LogVar("pid_attitude.pitch_outD", "Pitch attitude PID derivative term.", "Check pitch angle derivative/noise."),
    LogVar("pid_attitude.pitch_outFF", "Pitch attitude PID feed-forward term.", "Check pitch angle feed-forward."),
    LogVar("pid_attitude.yaw_outP", "Yaw attitude PID proportional term.", "Tune or inspect yaw angle loop."),
    LogVar("pid_attitude.yaw_outI", "Yaw attitude PID integral term.", "Check yaw angle integrator buildup."),
    LogVar("pid_attitude.yaw_outD", "Yaw attitude PID derivative term.", "Check yaw angle derivative/noise."),
    LogVar("pid_attitude.yaw_outFF", "Yaw attitude PID feed-forward term.", "Check yaw angle feed-forward."),

    # Rate PID terms
    LogVar("pid_rate.roll_outP", "Roll-rate PID proportional term.", "Tune or inspect roll-rate loop."),
    LogVar("pid_rate.roll_outI", "Roll-rate PID integral term.", "Check roll-rate integrator buildup."),
    LogVar("pid_rate.roll_outD", "Roll-rate PID derivative term.", "Check roll-rate derivative/noise."),
    LogVar("pid_rate.roll_outFF", "Roll-rate PID feed-forward term.", "Check roll-rate feed-forward."),
    LogVar("pid_rate.pitch_outP", "Pitch-rate PID proportional term.", "Tune or inspect pitch-rate loop."),
    LogVar("pid_rate.pitch_outI", "Pitch-rate PID integral term.", "Check pitch-rate integrator buildup."),
    LogVar("pid_rate.pitch_outD", "Pitch-rate PID derivative term.", "Check pitch-rate derivative/noise."),
    LogVar("pid_rate.pitch_outFF", "Pitch-rate PID feed-forward term.", "Check pitch-rate feed-forward."),
    LogVar("pid_rate.yaw_outP", "Yaw-rate PID proportional term.", "Tune or inspect yaw-rate loop."),
    LogVar("pid_rate.yaw_outI", "Yaw-rate PID integral term.", "Check yaw-rate integrator buildup."),
    LogVar("pid_rate.yaw_outD", "Yaw-rate PID derivative term.", "Check yaw-rate derivative/noise."),
    LogVar("pid_rate.yaw_outFF", "Yaw-rate PID feed-forward term.", "Check yaw-rate feed-forward."),

    # Motor requests
    LogVar("motor.m1req", "Motor 1 requested thrust after battery compensation, before PWM capping.", "Debug motor mixing/saturation.", True),
    LogVar("motor.m2req", "Motor 2 requested thrust after battery compensation, before PWM capping.", "Debug motor mixing/saturation.", True),
    LogVar("motor.m3req", "Motor 3 requested thrust after battery compensation, before PWM capping.", "Debug motor mixing/saturation.", True),
    LogVar("motor.m4req", "Motor 4 requested thrust after battery compensation, before PWM capping.", "Debug motor mixing/saturation.", True),
]


def print_section(title, rows):
    print(f"\n{title}")
    print("-" * len(title))
    for row in rows:
        print(f"{row.name}")
        print(f"  Definition: {row.definition}")
        print(f"  Use when:   {row.use_when}")


def main():
    most_useful = [row for row in LOG_VARS if row.useful]

    print_section("Most useful default PID log variables", most_useful)
    print_section("All default PID log variables in this reference", LOG_VARS)

    print("\nPython list: most_useful_pid_log_vars")
    print("most_useful_pid_log_vars = [")
    for row in most_useful:
        print(f"    {row.name!r},")
    print("]")


if __name__ == "__main__":
    main()



"""
There are two main state estimation Complementary Filter & Extended Kalman Filter. 

Complementary Filter takes the following sensors:

Internal sensors -> Gyroscope (x,y,z) & Accelerometers (x,y,z)
External sensors  -> (Zranger deck: Zranger):  Tof measurement (z) 

and outputs:
Attiude
Altitude

Extended Kalman Filter takes the following sensors:



Internal sensors -> Gyroscope (x,y,z) & Accelerometers (x,y,z)

** For the external sensors  there are 4 options to complete the setup
Flowdeck: Flow(x,y) , Tof measurement (z) 
FPSdeck: Relative Distance
Lighthouse 
Motion Capture: Position(x,y,z) & Angles(Pitch, Roll, Yaw)

and outputs:

Attitude (roll,pitch,yaw)
Position (x,y,z)
Velocity (x,y,z)


As far as controller.PID:
The firmware implements a three-layer cascaded PID architecture operating at approximately 
100 Hz, 500 Hz, and 500–1000 Hz, with state estimates provided by the onboard Kalman estimator. 

Desired Position -> [Position PID]  -> Desired Attitude  -> [Attitude PID] -> Desired Angular Rate -> [Rate PID] -> Motor Commands -> [Motor Distribution]


############ [Position PID]  ############

The position controller is not directly controlling motors. It is deciding what acceleration the droneshould have, and then converting that into the attitude and thrust needed to produce that acceleration.):

User Sets:
Desired X
Desired Y
Desired Z
Desired Yaw

Estimator Provides:
Estimated X
Estimated Y
Estimated Z
Estimated Yaw

Position PID:

(Desired X - Estimated X) → X Error → ax_des → Desired Pitch

(Desired Y - Estimated Y) → Y Error → ay_des → Desired Roll

(Desired Z - Estimated Z) → Z Error → az_des → Desired Thrust → Thrust_Command


############ [Attitude PID]  ############

The attitude controller receives the desired attitude from the
position controller and compares it against the estimated attitude.
It then determines how fast the drone should rotate in order to
reach the desired attitude.

Inputs:
Desired Pitch
Desired Roll
Desired Yaw

Estimator Provides:
Estimated Pitch
Estimated Roll
Estimated Yaw

Attitude PID:

(Desired Pitch - Estimated Pitch) → Pitch Error  → straight to PitchRate_ref (desired pitch rate)

(Desired Roll - Estimated Roll) → Roll Error. → straight to RollRate_ref (desired roll rate)

(Desired Yaw - Estimated Yaw) → Yaw Error → YawRate_ref (desired yaw rate)



############ [Rate PID]  ############

The rate controller receives the desired angular rates from the
attitude controller and compares them against the angular rates
measured by the gyroscope. It then determines the torque required
to achieve the desired rotation rates.

Inputs:
PitchRate_ref
RollRate_ref
YawRate_ref

Gyroscope Provides:
Measured Pitch Rate
Measured Roll Rate
Measured Yaw Rate

Rate PID:

(PitchRate_ref - Measured Pitch Rate) → Pitch Rate Error → Pitch Torque Command

(RollRate_ref - Measured Roll Rate) → Roll Rate Error → Roll Torque Command

(YawRate_ref - Measured Yaw Rate) → Yaw Rate Error → Yaw Torque Command


############ [Motor Distribution or Mixer] ############

Motor 1 Command = Thrust_Command + Roll Torque Command + Pitch Torque Command + Yaw Torque Command

Motor 2 Command = Thrust_Command - Roll Torque Command + Pitch Torque Command - Yaw Torque Command

Motor 3 Command = Thrust_Command - Roll Torque Command - Pitch Torque Command + Yaw Torque Command

Motor 4 Command = Thrust_Command + Roll Torque Command - Pitch Torque Command - Yaw Torque Command

"""