import asyncio
import moteus
import signal
import sys
import pandas as pd
import os
import datetime
import argparse
from speed_log_parser import FieldType, LoggedField, decode_speedlog, row_size


def calculate_decimation(buf_size, log_fields, base_speed, periods, isr_speed=30000) -> tuple[int, float, float]:
    """Calculate the decimation and row duration for the speed logger."""
    samples = buf_size // row_size(log_fields)
    duration = periods / base_speed  # seconds for 'periods' periods
    decimation = max(1, round(duration * isr_speed / samples))

    return decimation, decimation / isr_speed, samples * decimation / isr_speed


def plot_results(data: pd.DataFrame, save_file=None, show_plot=True):
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    axs[0].scatter(data["time_s"], data["velocity"], label="Measured velocity")
    axs[0].scatter(data["time_s"], data["velocity_setpoint"], label="Desired velocity")
    axs[0].set_ylabel("Velocity [rps]")
    axs[0].set_title("Velocity tracking")
    axs[0].legend()
    axs[0].grid(True)

    axs[1].scatter(data["time_s"], data["torque"], label="Measured torque")
    axs[1].scatter(data["time_s"], data["torque_pi"], label="PI regulator torque")
    axs[1].scatter(data["time_s"], data["torque_setpoint"], label="Total desired torque")
    axs[1].set_ylabel("Torque [Nm]")
    axs[1].set_xlabel("Time [s]")
    axs[1].set_title("Torque")
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()

    if save_file:
        plt.savefig(save_file, dpi=300)
        print(f"Plot saved to {save_file}")

    if show_plot:
        plt.show()

    plt.close(fig)

def set_signal_handler(loop, stop_event, controller):
    first = True

    def handler():
        nonlocal first
        if first:
            first = False
            print("Signal received, stopping...")
            stop_event.set()
            controller.set_stop()
        else:
            print("Second signal received, exiting immediately")
            loop.stop()
            controller.set_stop()
            sys.exit(1)
    loop.add_signal_handler(signal.SIGINT, handler)


async def run_test(args):
    transport = moteus.PythonCan(interface="socketcan", channel="vcan1", fd=True)
    controller = moteus.Controller(id=1, transport=transport)
    command_stream = moteus.Stream(controller)

    # Ensure motor is stopped before starting
    await controller.set_stop()

    # Setup signal handling for graceful shutdown
    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()
    set_signal_handler(loop, stop_event, controller)

    fields = [
        LoggedField("velocity", FieldType.FLOAT32),
        LoggedField("velocity_setpoint", FieldType.FLOAT32),
        LoggedField("torque_pi", FieldType.FLOAT32),
        LoggedField("torque_setpoint", FieldType.FLOAT32),
        LoggedField("torque", FieldType.FLOAT32),
    ]

    decimation, row_duration, total_duration = calculate_decimation(
        args.buf_size, fields, args.base_speed, args.periods
    )

    await command_stream.command(f"conf set speed_logger.size {args.buf_size}".encode("utf-8"))
    await command_stream.command(f"conf set speed_logger.decimation {decimation}".encode("utf-8"))
    print(f"Configured logger with decimation {decimation}, row duration {row_duration:.4f} s")

    # Ask user to confirm, print all settings over multiple lines
    print(f"Ready to start sinusoidal velocity command with the following settings:"
          f"\n  Base speed     = {args.base_speed} rps"
          f"\n  Amplitude      = {args.amplitude}"
          f"\n  Max torque     = {args.max_torque} Nm"
          f"\n  Buffer size    = {args.buf_size} bytes"
          f"\n  Periods        = {args.periods}"
          f"\n  Start delay    = {args.start_delay} s"
          )

    print("WARNING: Rotor will move! Ensure it is safe to do so.")
    input("Press Enter to continue...")

    await command_stream.command(f"d sinvel {args.base_speed:.3f} {args.amplitude:.3f} {args.max_torque:.3f}".encode("utf-8"))
    await asyncio.sleep(args.start_delay)

    await command_stream.command(f"d start_logger".encode("utf-8"))
    await asyncio.sleep(total_duration)
    await command_stream.command(f"d stop_logger".encode("utf-8"))
    await command_stream.command(f"d stop".encode("utf-8"))

    # Get the results
    res = await command_stream.command(b"d speed_log read")
    data = decode_speedlog(res.decode("utf-8"), fields)

    # Append a "time (s)" column
    data.insert(0, "time_s", [i * row_duration for i in range(len(data))])

    plot_results(data)


def main():
    parser = argparse.ArgumentParser(description="Run a sinusoidal velocity test with moteus speed logger.")
    parser.add_argument("--base-speed", type=float, required=True,
                        help="Rotation speed in rps (required).")
    parser.add_argument("--amplitude", type=float, required=True,
                        help="Sinusoidal velocity amplitude as fraction of base speed (< 1.0).")
    parser.add_argument("--max-torque", type=float, default=0.5,
                        help="Maximum torque [Nm] (default: 0.5).")
    parser.add_argument("--buf-size", type=int, default=8192,
                        help="Logger buffer size in bytes (default: 8192).")
    parser.add_argument("--periods", type=int, default=3,
                        help="Number of periods to capture in the buffer (default: 3).")
    parser.add_argument("--start-delay", type=float, default=5.0,
                        help="Delay for rotor to spin up before logging starts [s] (default: 5.0).")
    parser.add_argument("--hide-plot", action="store_true",
                        help="Do not display plot (useful in batch runs).")
    parser.add_argument("--save-plot", nargs="?", const=True, metavar="FILENAME",
                        help="Save plot to file. If no filename is given, one is generated automatically.")
    parser.add_argument("--plot-input", type=str, metavar="CSV",
                        help="Load data from CSV file instead of running the test.")
    parser.add_argument("--save-data", nargs="?", const=True, metavar="CSV",
                        help="Save logged data to CSV file. If no filename is given, one is generated automatically.")

    args = parser.parse_args()

    if args.plot_input:
        data = pd.read_csv(args.plot_input)
    else:
        data = asyncio.run(run_test(args))

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    # Save data if requested
    if args.save_data:
        if args.save_data is True:
            filename = f"speedlog_{timestamp}.csv"
        else:
            filename = args.save_data
        data.to_csv(filename, index=False)
        print(f"Data saved to {filename}")

    # Save / show plot
    if args.save_plot:
        if args.save_plot is True:
            plot_file = f"speedlog_{timestamp}.png"
        else:
            plot_file = args.save_plot
    else:
        plot_file = None

    plot_results(data, save_file=plot_file, show_plot=not args.hide_plot)


if __name__ == "__main__":
    main()