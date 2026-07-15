import csv
import socket
import time
from datetime import datetime

UR_IP = "192.168.1.102"
UR_PORT = 30003

CSV_FILENAME = "elastic_characterization_robot_control.csv"

STEP_DISTANCE = 0.001   # 1 mm
NUMBER_OF_STEPS = 20
STEP_INTERVAL = 5.0     # seconds
FINAL_WAIT = 10.0       # seconds

DOWN_ACCELERATION = 0.05
DOWN_VELOCITY = 0.002

RETURN_ACCELERATION = 0.10
RETURN_VELOCITY = 0.01


def build_script() -> str:
    """
    The robot:
      1. Saves its current TCP pose internally.
      2. Moves downward by 1 mm.
      3. Waits 5 seconds.
      4. Repeats until reaching 20 mm.
      5. Waits 10 seconds.
      6. Returns to the saved initial pose.

    No robot-to-computer feedback connection is used.
    """

    return f"""
def elastic_characterization():

  initial_pose = get_actual_tcp_pose()

  step = 1

  while step <= {NUMBER_OF_STEPS}:

    target_pose = initial_pose
    target_pose[2] = initial_pose[2] - step * {STEP_DISTANCE}

    movel(
      target_pose,
      a={DOWN_ACCELERATION},
      v={DOWN_VELOCITY},
      r=0
    )

    sleep({STEP_INTERVAL})

    step = step + 1
  end

  sleep({FINAL_WAIT})

  movel(
    initial_pose,
    a={RETURN_ACCELERATION},
    v={RETURN_VELOCITY},
    r=0
  )

end
"""


def write_csv_log() -> None:
    """
    Create a nominal command log based on the programmed timing.

    Because there is no feedback, these timestamps represent when the
    commands are expected to occur, not confirmed robot arrival times.
    """

    start_time = time.monotonic()

    with open(CSV_FILENAME, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)

        writer.writerow([
            "timestamp",
            "elapsed_time_s",
            "event",
            "commanded_downward_displacement_m",
            "commanded_downward_displacement_mm",
        ])

        writer.writerow([
            datetime.now().isoformat(timespec="milliseconds"),
            "0.000",
            "initial",
            "0.000000",
            "0.000",
        ])
        csv_file.flush()

        for step in range(1, NUMBER_OF_STEPS + 1):
            # The robot first performs the movement, then starts its 5 s wait.
            # With no feedback, Python cannot know the exact move completion time.
            displacement_m = step * STEP_DISTANCE

            writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"),
                f"{time.monotonic() - start_time:.3f}",
                f"step_{step}",
                f"{displacement_m:.6f}",
                f"{displacement_m * 1000.0:.3f}",
            ])
            csv_file.flush()

            time.sleep(STEP_INTERVAL)

        writer.writerow([
            datetime.now().isoformat(timespec="milliseconds"),
            f"{time.monotonic() - start_time:.3f}",
            "maximum_displacement_wait",
            f"{NUMBER_OF_STEPS * STEP_DISTANCE:.6f}",
            f"{NUMBER_OF_STEPS * STEP_DISTANCE * 1000.0:.3f}",
        ])
        csv_file.flush()

        time.sleep(FINAL_WAIT)

        writer.writerow([
            datetime.now().isoformat(timespec="milliseconds"),
            f"{time.monotonic() - start_time:.3f}",
            "return_to_initial",
            "0.000000",
            "0.000",
        ])
        csv_file.flush()


def main() -> None:
    script = build_script()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10.0)

    try:
        sock.connect((UR_IP, UR_PORT))
        sock.sendall(script.encode("utf-8"))

        print(f"URScript sent to {UR_IP}:{UR_PORT}")
        print("Robot motion sequence started.")

    except (ConnectionRefusedError, TimeoutError, socket.timeout) as error:
        print(f"Could not connect to the robot: {error}")
        return

    except OSError as error:
        print(f"Socket error: {error}")
        return

    finally:
        sock.close()

    # Generate a feedforward command log.
    write_csv_log()

    print(f"Command log saved to: {CSV_FILENAME}")


if __name__ == "__main__":
    main()