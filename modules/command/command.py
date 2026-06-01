"""
Decision-making logic.
"""

import math

from pymavlink import mavutil

from ..common.modules.logger import logger
from ..telemetry import telemetry


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on recieved telemetry,
    and send out commands based upon the data.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> "tuple[True, Command] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Command object.
        """
        #  Create a Command object
        try:
            return True, Command(cls.__private_key, connection, target, local_logger)
        except Exception as e:
            local_logger.error(f"Failed to create Command object: {e}", True)
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        # Do any intializiation here
        self.connection = connection
        self.target = target
        self.local_logger = local_logger

        self.vx_sum = 0.0
        self.vy_sum = 0.0
        self.vz_sum = 0.0
        self.sample_count = 0

    def run(
        self,
        telemetry_data: telemetry.TelemetryData
    ) -> "tuple[bool, str | None]":
        """
        Make a decision based on received telemetry data.
        """
        if telemetry_data is None:
            self.local_logger.error("Received None telemetry data")
            return False, None
        
        # Log average velocity for this trip so far
        self.vx_sum += telemetry_data.x_velocity
        self.vy_sum += telemetry_data.y_velocity
        self.vz_sum += telemetry_data.z_velocity
        self.sample_count += 1

        avg_vx = self.vx_sum / self.sample_count
        avg_vy = self.vy_sum / self.sample_count
        avg_vz = self.vz_sum / self.sample_count

        self.local_logger.info(
            f"Average velocity so far: ({avg_vx:.3f}, {avg_vy:.3f}, {avg_vz:.3f}) m/s"
        )

        output_string = None

        # Use COMMAND_LONG (76) message, assume the target_system=1 and target_componenet=0
        # The appropriate commands to use are instructed below

        # Adjust height using the comand MAV_CMD_CONDITION_CHANGE_ALT (113)
        # String to return to main: "CHANGE_ALTITUDE: {amount you changed it by, delta height in meters}"

        delta_z = self.target.z - telemetry_data.z
        if abs(delta_z) > 0.5:
            self.connection.mav.command_long_send(
                1,
                0,
                mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                self.target.z,
            )
            output_string = f"CHANGE_ALTITUDE: {delta_z}"
            self.local_logger.info(output_string)
            return True, output_string

        # Adjust direction (yaw) using MAV_CMD_CONDITION_YAW (115). Must use relative angle to current state
        # String to return to main: "CHANGING_YAW: {degree you changed it by in range [-180, 180]}"
        # Positive angle is counter-clockwise as in a right handed system
        dx = self.target.x - telemetry_data.x
        dy = self.target.y - telemetry_data.y
        desired_yaw_rad = math.atan2(dy, dx)

        delta_rad = desired_yaw_rad - telemetry_data.yaw
        delta_deg = math.degrees(delta_rad)

        while delta_deg > 180:
            delta_deg -= 360
        while delta_deg < -180:
            delta_deg += 360
        
        if abs(delta_deg) > 5.0:
            self.connection.mav.command_long_send(
                1,
                0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                delta_deg,
                5,
                0,
                1,
                0,
                0,
                0,
            )
            output_string = f"CHANGING_YAW: {delta_deg}"
            self.local_logger.info(output_string)
            return True, output_string

        return True, None


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
