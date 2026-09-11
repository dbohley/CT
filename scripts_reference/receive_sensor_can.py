import struct
import can

# ==============================================================================
# CONFIGURATION
# ==============================================================================
# Set your serial port (Windows: 'COM3', 'COM4', etc. | Linux: '/dev/ttyACM0')
SERIAL_PORT = 'COM13'  
BITRATE = 1000000      # 500 kbps (must match the Teensy speed)

def main():
    print(f"Connecting to RH-02 PLUS on {SERIAL_PORT} @ {BITRATE} bps...")
    sensor_filters = [
    {"can_id": 0x5, "can_mask": 0x7FF, "extended": False}
    ]
    try:
        # Open SLCAN interface via virtual serial port
        bus = can.Bus(
            interface='slcan', 
            channel=SERIAL_PORT, 
            bitrate=BITRATE,
            can_filters=sensor_filters
        )
    except Exception as e:
        print(f"\n[ERROR] Could not open CAN interface: {e}")
        print("Check that the COM port is correct and not open in another application.")
        return

    print("Listening for CAN ID 5 messages... (Press Ctrl+C to exit)\n")

    try:
        for msg in bus:
            # Filter specifically for CAN ID 5 with 8 bytes payload
            if msg.arbitration_id == 5 and len(msg.data) == 8:
                
                # Unpack 8 bytes payload binary structure:
                # '<'  = Little-endian
                # 'f'  = float (4 bytes) -> ToF Distance (mm)
                # 'f'  = float (4 bytes) -> Calculated Distance (cm)
                tof_dist_mm, calc_dist_cm = struct.unpack('<ff', msg.data)

                # Output formatted data
                print(f"[CAN ID 5] Time: {msg.timestamp:.3f}s")
                print(f"  ├── ToF Distance:        {tof_dist_mm:.3f} mm")
                print(f"  └── Calculated Distance: {calc_dist_cm:.3f} cm")
                print("-" * 45)

    except KeyboardInterrupt:
        print("\nStopping CAN receiver...")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()