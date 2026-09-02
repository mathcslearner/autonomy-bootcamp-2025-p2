"""
Telemetry gathering logic.
"""

import time

from pymavlink import mavutil

from ..common.modules.logger import logger


class TelemetryData:  # pylint: disable=too-many-instance-attributes
    """
    Python struct to represent Telemtry Data. Contains the most recent attitude and position reading.
    """

    def __init__(
        self,
        time_since_boot: int | None = None,  # ms
        x: float | None = None,  # m
        y: float | None = None,  # m
        z: float | None = None,  # m
        x_velocity: float | None = None,  # m/s
        y_velocity: float | None = None,  # m/s
        z_velocity: float | None = None,  # m/s
        roll: float | None = None,  # rad
        pitch: float | None = None,  # rad
        yaw: float | None = None,  # rad
        roll_speed: float | None = None,  # rad/s
        pitch_speed: float | None = None,  # rad/s
        yaw_speed: float | None = None,  # rad/s
    ) -> None:
        self.time_since_boot = time_since_boot
        self.x = x
        self.y = y
        self.z = z
        self.x_velocity = x_velocity
        self.y_velocity = y_velocity
        self.z_velocity = z_velocity
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.roll_speed = roll_speed
        self.pitch_speed = pitch_speed
        self.yaw_speed = yaw_speed

    def __str__(self) -> str:
        return f"""{{
            time_since_boot: {self.time_since_boot},
            x: {self.x},
            y: {self.y},
            z: {self.z},
            x_velocity: {self.x_velocity},
            y_velocity: {self.y_velocity},
            z_velocity: {self.z_velocity},
            roll: {self.roll},
            pitch: {self.pitch},
            yaw: {self.yaw},
            roll_speed: {self.roll_speed},
            pitch_speed: {self.pitch_speed},
            yaw_speed: {self.yaw_speed}
        }}"""


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Telemetry:
    """
    Telemetry class to read position and attitude (orientation).
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[True, Telemetry] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Telemetry object.
        """
        # Create a Telemetry object
        try:
            return True, Telemetry(cls.__private_key, connection, local_logger)

        except (OSError, TypeError, AttributeError) as e:
            local_logger.error(f"Failed to create Telemetry object: {e}", True)
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Telemetry.__private_key, "Use create() method"

        # Do any intializiation here
        self.connection = connection
        self.logger = local_logger

    def run(
        self,
    ) -> "tuple[bool, TelemetryData | None]":
        """
        Receive LOCAL_POSITION_NED and ATTITUDE messages from the drone,
        combining them together to form a single TelemetryData object.
        """
        # Read MAVLink message LOCAL_POSITION_NED (32)
        # Read MAVLink message ATTITUDE (30)
        # Return the most recent of both, and use the most recent message's timestamp
        start_time = time.time()

        # Both must be received fresh within this cycle; stale values are not reused.
        attitude = None
        local_position = None

        while time.time() - start_time <= 1:
            msg = self.connection.recv_match(type=["ATTITUDE", "LOCAL_POSITION_NED"], timeout=0.1)

            if not msg:
                continue
            if msg.get_type() == "LOCAL_POSITION_NED":
                local_position = msg
            elif msg.get_type() == "ATTITUDE":
                attitude = msg

            # Stop only once this cycle has a fresh reading of both message types.
            if attitude is not None and local_position is not None:
                break

        # A cycle that did not receive fresh data for both types could not get a
        # complete reading; report failure so the worker can log it.
        if attitude is None or local_position is None:
            return False, None

        return_telemetry = TelemetryData()
        return_telemetry.time_since_boot = max(local_position.time_boot_ms, attitude.time_boot_ms)
        return_telemetry.roll = attitude.roll
        return_telemetry.pitch = attitude.pitch
        return_telemetry.yaw = attitude.yaw
        return_telemetry.roll_speed = attitude.rollspeed
        return_telemetry.pitch_speed = attitude.pitchspeed
        return_telemetry.yaw_speed = attitude.yawspeed
        return_telemetry.x = local_position.x
        return_telemetry.y = local_position.y
        return_telemetry.z = local_position.z
        return_telemetry.x_velocity = local_position.vx
        return_telemetry.y_velocity = local_position.vy
        return_telemetry.z_velocity = local_position.vz
        return True, return_telemetry


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
