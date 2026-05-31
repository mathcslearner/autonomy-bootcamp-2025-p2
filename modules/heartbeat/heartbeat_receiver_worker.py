"""
Heartbeat worker that sends heartbeats periodically.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import heartbeat_receiver
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def heartbeat_receiver_worker(
    connection: mavutil.mavfile,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    Connection = mavlink connection to listen for heartbeats
    output_queue = queue to report connection state strings
    controller = workercontroller used to signal it to stop
    """
    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    # Instantiate logger
    worker_name = pathlib.Path(__file__).stem
    process_id = os.getpid()
    result, local_logger = logger.Logger.create(f"{worker_name}_{process_id}", True)
    if not result:
        print("ERROR: Worker failed to create logger")
        return

    # Get Pylance to stop complaining
    assert local_logger is not None

    local_logger.info("Logger initialized", True)

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate class object (heartbeat_receiver.HeartbeatReceiver)
    result, receiver = heartbeat_receiver.HeartbeatReceiver.create(connection, local_logger)
    if not result:
        local_logger.error("Failed to create HeartbeatReceiver, exiting worker", True)
        return
    
    # Get Pylance to stop complaining
    assert receiver is not None

    local_logger.info("HeartbeatReceiver created successfully", True)

    # Main loop: do work.
    while not controller.is_exit_requested():
        controller.check_pause()
        result = receiver.run()
        if not result:
            local_logger.error("HeartbeatReceiver.run() failed unexpectedly", True)
            continue
    
        output_queue.queue.put("Connected" if receiver.is_connected else "Disconnected")
    
    local_logger.info("Heartbeat receiver worker exiting")


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
