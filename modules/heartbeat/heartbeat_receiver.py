"""
Heartbeat receiving logic.
"""

from pymavlink import mavutil

from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatReceiver:
    """
    HeartbeatReceiver class to send a heartbeat
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[True, HeartbeatReceiver] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        # Create a HeartbeatReceiver object
        if connection is None:
            local_logger.error("Connection is None, cannot create HeartbeatReceiver")
            return False, None

        return True, HeartbeatReceiver(cls.__private_key, connection, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        # Do any intializiation here
        self.connection = connection
        self.logger = local_logger

        # Number of consecutive missed heartbeats
        self.missed_count = 0
        # Start disconnected until first heartbeat is received
        self.is_connected = False

    def run(
        self,
    ) -> bool:
        """
        Attempt to recieve a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.
        """
        try:
            message = self.connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        except (OSError, TypeError, AttributeError) as e:
            self.logger.error(f"Unexpected error while receiving heartbeat: {e}", True)
            return False

        if message is None:
            # No heartbeat received within the timeout window
            self.missed_count += 1
            self.logger.warning(f"Missed heartbeat (consecutive misses: {self.missed_count})", True)

            if self.missed_count >= 5:
                self.is_connected = False

        else:
            # Successfully received a heartbeat
            self.missed_count = 0
            self.is_connected = True

        return True


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
