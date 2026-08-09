"""
Telemtry worker that gathers GPS data.
"""

import os
import pathlib
import queue as py_queue

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import telemetry
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def telemetry_worker(
    connection: mavutil.mavfile,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    connection: mavlink connection to drone
    output_queue: output queue to send messages
    controller: shared controller to signal to worker

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
    # Instantiate class object (telemetry.Telemetry)
    result, telemetry_instance = telemetry.Telemetry.create(connection, local_logger)
    if not result:
        local_logger.error("Failed to create Telemetry instance")
        return

    assert telemetry_instance is not None

    # Main loop: do work.
    while not controller.is_exit_requested():
        controller.check_pause()
        result, telemetry_data = telemetry_instance.run()
        if not result:
            local_logger.warning(
                "Telemetry run() failed (timeout or bad message), retrying...", True
            )
            continue

        try:
            output_queue.queue.put(telemetry_data, timeout=0.1)
        except py_queue.Full:
            local_logger.warning("Telemetry output queue full; dropping sample", True)
            continue

        local_logger.info(f"Telemetry data enqueued: {telemetry_data}")


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
