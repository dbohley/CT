import can
import struct
import time
import csv
import numpy as np

# ============================================================
# Motor / CAN configuration
# ============================================================

MOTOR_ID = 1
CMD_VELOCITY = 3



bus = can.Bus(
    interface="slcan",
    channel="COM3",
    bitrate=1000000,
)

# ============================================================
# Breathing-profile configuration
# ============================================================

CSV_FILE = "breathing_profile.csv"

# Linear distance traveled by the mechanism for one full rotation
# of the OUTPUT shaft (after gearbox reduction).
MOTOR_MM_PER_REV = 5.0

# Gearbox reduction ratio (6:1 means 6 motor rotor turns = 1 output turn)
GEAR_RATIO = 6.0

# CubeMars AK60-6 V3 internal motor pole pair count (28 poles / 2)
MOTOR_POLE_PAIRS = 14

# Change to -1 if the motor moves opposite to the CSV profile.
DIRECTION = 1

# How often to update the motor command.
CONTROL_DT = 0.01  # 10 ms = 100 Hz


# Speed limit configuration
MAX_OUTPUT_RPM = 60.0
MAX_ERPM = int(MAX_OUTPUT_RPM * GEAR_RATIO * MOTOR_POLE_PAIRS)  # 5,040 ERPM

def set_velocity_erpm(motor_id: int, erpm: int):
    """Send a velocity command to the motor."""
    can_id = (CMD_VELOCITY << 8) | motor_id
    data = struct.pack(">i", int(erpm))
    msg = can.Message(
        arbitration_id=can_id,
        data=data,
        is_extended_id=True,
    )
    bus.send(msg)


def load_breathing_profile(filename):
    """
    Load a CSV containing:
        time_s,y_mm

    Returns numpy arrays of time and position.
    """
    times = []
    positions = []

    with open(filename, "r", newline="") as f:
        reader = csv.DictReader(f)

        if "time_s" not in reader.fieldnames or "y_mm" not in reader.fieldnames:
            raise ValueError(
                "CSV must contain columns named exactly 'time_s' and 'y_mm'."
            )

        for row in reader:
            times.append(float(row["time_s"]))
            positions.append(float(row["y_mm"]))

    if len(times) < 2:
        raise ValueError("The breathing profile must contain at least 2 points.")

    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)

    # Make sure time is increasing.
    order = np.argsort(times)
    times = times[order]
    positions = positions[order]

    if np.any(np.diff(times) <= 0):
        raise ValueError("time_s values must be strictly increasing.")

    return times, positions


def position_to_velocity_erpm(times, positions):
    """
    Convert CSV position profile y(t) in mm into ERPM required by the controller,
    clamped to MAX_OUTPUT_RPM.
    """
    # Smooth numerical derivative using numpy.gradient.
    velocity_mm_s = np.gradient(positions, times)

    # Linear speed -> output shaft RPM.
    output_rpm = (velocity_mm_s / MOTOR_MM_PER_REV) * 60.0

    # Output RPM -> motor rotor RPM.
    rotor_rpm = output_rpm * GEAR_RATIO

    # Rotor RPM -> electrical RPM (ERPM).
    erpm = rotor_rpm * MOTOR_POLE_PAIRS * DIRECTION

    # Clamp ERPM to [-MAX_ERPM, MAX_ERPM]
    erpm_clamped = np.clip(erpm, -MAX_ERPM, MAX_ERPM)

    return np.rint(erpm_clamped).astype(np.int32)


def run_breathing_profile():
    times, positions = load_breathing_profile(CSV_FILE)

    # Calculate the motor velocity corresponding to every CSV point.
    erpm_profile = position_to_velocity_erpm(times, positions)

    print(f"Loaded {len(times)} breathing-profile points.")
    print(f"Duration: {times[-1] - times[0]:.3f} s")
    print(f"Position range: {positions.min():.3f} to {positions.max():.3f} mm")
    print(
        f"Commanded ERPM range: "
        f"{erpm_profile.min()} to {erpm_profile.max()}"
    )

    # Shift time so the profile starts at t=0.
    times = times - times[0]

    start = time.monotonic()

    try:
        i = 0

        while True:
            elapsed = time.monotonic() - start

            if elapsed >= times[-1]:
                break

            # Find the two CSV points surrounding the current time
            # and interpolate between their velocity commands.
            while i < len(times) - 2 and times[i + 1] < elapsed:
                i += 1

            commanded_erpm = np.interp(
                elapsed,
                times,
                erpm_profile,
            )

            set_velocity_erpm(MOTOR_ID, int(round(commanded_erpm)))

            # Keep the control loop approximately at CONTROL_DT.
            next_time = start + elapsed + CONTROL_DT
            sleep_time = next_time - time.monotonic()

            if sleep_time > 0:
                time.sleep(sleep_time)

    finally:
        # Always stop the motor if the profile ends or an exception occurs.
        set_velocity_erpm(MOTOR_ID, 0)
        time.sleep(0.1)
        bus.shutdown()


if __name__ == "__main__":
    run_breathing_profile()