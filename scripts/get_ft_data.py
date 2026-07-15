#!/usr/bin/env python3

import sys
import numpy as np
import nidaqmx

from nidaqmx.constants import (
    AcquisitionType,
    TerminalConfiguration,
)
from nidaqmx.stream_readers import AnalogMultiChannelReader


SAMPLE_RATE = 1000.0       # Samples per second, per channel
BLOCK_SIZE = 100           # Samples read from each channel per block

FIRST_CHANNEL = 0
NUMBER_OF_CHANNELS = 6

MIN_VOLTAGE = -10.0
MAX_VOLTAGE = 10.0


def find_usb_6341() -> str:
    """Find the first connected NI USB-6341 and return its DAQmx name."""

    system = nidaqmx.system.System.local()
    devices = list(system.devices)

    if not devices:
        raise RuntimeError(
            "NI-DAQmx found no devices. Check that the NI-DAQmx driver "
            "is installed and that the DAQ is accessible."
        )

    print("DAQmx devices found:", file=sys.stderr)

    for device in devices:
        print(
            f"  {device.name}: {device.product_type}, "
            f"serial={device.dev_serial_num}",
            file=sys.stderr,
        )

        if "USB-6341" in device.product_type.upper():
            return device.name

    raise RuntimeError("No NI USB-6341 was found by NI-DAQmx.")


def main() -> None:
    device_name = find_usb_6341()

    channel_string = (
        f"{device_name}/ai{FIRST_CHANNEL}:"
        f"{FIRST_CHANNEL + NUMBER_OF_CHANNELS - 1}"
    )

    print(
        f"Reading {channel_string} at {SAMPLE_RATE:.1f} samples/s/channel",
        file=sys.stderr,
    )
    print("Press Ctrl+C to stop.", file=sys.stderr)

    # Shape is: [number of channels, samples per channel]
    data = np.empty(
        (NUMBER_OF_CHANNELS, BLOCK_SIZE),
        dtype=np.float64,
    )

    try:
        with nidaqmx.Task() as task:
            task.ai_channels.add_ai_voltage_chan(
                physical_channel=channel_string,
                terminal_config=TerminalConfiguration.DIFFERENTIAL,
                min_val=MIN_VOLTAGE,
                max_val=MAX_VOLTAGE,
            )

            task.timing.cfg_samp_clk_timing(
                rate=SAMPLE_RATE,
                sample_mode=AcquisitionType.CONTINUOUS,
                samps_per_chan=BLOCK_SIZE * 10,
            )

            reader = AnalogMultiChannelReader(task.in_stream)

            task.start()

            while True:
                samples_read = reader.read_many_sample(
                    data=data,
                    number_of_samples_per_channel=BLOCK_SIZE,
                    timeout=2.0,
                )

                # data[:, sample_index] gives one simultaneous six-channel scan.
                for sample_index in range(samples_read):
                    sample = tuple(
                        float(data[channel, sample_index])
                        for channel in range(NUMBER_OF_CHANNELS)
                    )

                    print(
                        "(" + ", ".join(f"{value:.6f}" for value in sample) + ")"
                    )

    except KeyboardInterrupt:
        print("\nAcquisition stopped.", file=sys.stderr)

    except nidaqmx.errors.DaqError as error:
        print(f"NI-DAQmx error:\n{error}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()