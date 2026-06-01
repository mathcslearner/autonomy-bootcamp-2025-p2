"""
Command worker to make decisions based on Telemetry Data.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import command
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def command_worker(
    connection: mavutil.mavfile,
    target: command.Position,
    input_queue: queue_proxy_wrapper.QueueProxyWrapper,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    connection: mavlink connection to drone
    target: 3D target position
    input_queue: input queue receiving telemetry data from telemetry worker
    output_queue: output queue sending result strings to main process
    controller: shared controller
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
    # Instantiate class object (command.Command)
    result, command_instance = command.Command.create(connection, target, local_logger)
    if not result:
        local_logger.error("Failed to create Command instance", True)
        return

    assert command_instance is not None

    # Main loop: do work.
    while not controller.is_exit_requested():
        controller.check_pause()

        if input_queue.queue.empty():
            continue

        telemetry_data = input_queue.queue.get()

        result, output = command_instance.run(telemetry_data)
        if not result:
            local_logger.warning("Command run() failed, skipping this telemetry sample")
            continue

        if output is not None:
            output_queue.queue.put(output)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
