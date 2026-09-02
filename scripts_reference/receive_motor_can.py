import can
import struct
import time
import logging
import argparse
import threading

# --- Configure Logging ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(threadName)s | %(message)s',
    datefmt='%H:%M:%S',
    handlers=[
        logging.FileHandler("dual_motor_output.log"),
        logging.StreamHandler()
    ]
)

P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -30.0, 30.0
T_MIN, T_MAX = -100.0, 100.0

def uint_to_float(x_int, x_min, x_max, bits):
    span = (1 << bits) - 1
    return ((float(x_int) / span) * (x_max - x_min)) + x_min

def parse_feedback(data):
    if len(data) < 8: return None
    motor_id = data[0] & 0x0F
    error_code = data[0] >> 4
    pos_int = (data[1] << 8) | data[2]
    vel_int = (data[3] << 4) | (data[4] >> 4)
    torque_int = ((data[4] & 0x0F) << 8) | data[5]
    
    return {
        "id": motor_id, "error": error_code,
        "pos": uint_to_float(pos_int, P_MIN, P_MAX, 16),
        "vel": uint_to_float(vel_int, V_MIN, V_MAX, 12),
        "torque": uint_to_float(torque_int, T_MIN, T_MAX, 12)
    }

def hold_ak_motor(bus_channel, bus_bitrate, motor_id, master_id):
    """Keeps the AK motor stale in place by holding its initial position."""
    threading.current_thread().name = f"AK_Motor_ID{motor_id}"
    try:
        bus = can.interface.Bus(bustype='slcan', channel=bus_channel, bitrate=bus_bitrate)
    except Exception as e:
        logging.error(f"Init Error on {bus_channel}: {e}")
        return

    control_id = 0x100 + motor_id
    current_pos = 0.0
    
    # 1. Listen to the bus briefly to find its current position before commanding it
    # logging.info("Listening for initial position to hold...")
    # start_time = time.time()
    # while time.time() - start_time < 2.0:
    #     msg = bus.recv(0.1)
    #     if msg and msg.arbitration_id == master_id:
    #         parsed = parse_feedback(msg.data)
    #         if parsed and parsed['id'] == motor_id:
    #             current_pos = parsed['pos']
    #             logging.info(f"Initial position found: {current_pos:.2f} rad. Holding here.")
    #             break

    # 2. Command it to stay at that position with 0 velocity
    hold_data = struct.pack('<ff', current_pos, 0.2)
    hold_msg = can.Message(arbitration_id=control_id, data=hold_data, is_extended_id=False)

    try:
        while True:
            bus.send(hold_msg)
            msg = bus.recv(0.05)
            if msg and msg.arbitration_id == master_id:
                parsed = parse_feedback(msg.data)
                if parsed and parsed['id'] == motor_id and parsed['error'] != 0:
                    logging.warning(f"FAULT! Error Code: {hex(parsed['error'])}")
            time.sleep(0.01)
    except Exception as e:
        logging.error(f"Error: {e}")
    finally:
        bus.shutdown()

def drive_motor(bus_channel, bus_bitrate, motor_id, master_id, p_des, v_des):
    """Drives the GL40 II motor to a target position."""
    threading.current_thread().name = f"GL40_ID{motor_id}"
    try:
        bus = can.interface.Bus(bustype='slcan', channel=bus_channel, bitrate=bus_bitrate)
    except Exception as e:
        logging.error(f"Init Error on {bus_channel}: {e}")
        return

    control_id = 0x100 + motor_id
    control_data = struct.pack('<ff', p_des, v_des)
    control_msg = can.Message(arbitration_id=control_id, data=control_data, is_extended_id=False)

    state = {'pos': 0.0, 'id': motor_id, 'error': 0}
    logging.info(f"Target -> Pos: {p_des} rad, Vel: {v_des} rad/s")
    
    try:
        while True:
            bus.send(control_msg)
            msg = bus.recv(0.05)
            if msg and msg.arbitration_id == master_id:
                parsed = parse_feedback(msg.data)
                if parsed and parsed['id'] == motor_id:
                    state = parsed 
                    if state['error'] != 0:
                        logging.warning(f"FAULT! Error Code: {hex(state['error'])}")
                    else:
                        logging.info(f"Pos: {state['pos']:.4f} | Vel: {state['vel']:.4f} | Torque: {state['torque']:.4f}")            

            time.sleep(0.01) 
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = can.Message(arbitration_id=control_id, data=struct.pack('<ff', state['pos'], 0.0), is_extended_id=False)
        bus.send(stop_msg)
        bus.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dual Motor Control via CAN")
    
    # GL40 II Arguments
    parser.add_argument('--gl_channel', type=str, default='COM10')
    parser.add_argument('--gl_id', type=int, default=2)
    parser.add_argument('--pos', type=float, default=1.0)
    parser.add_argument('--vel', type=float, default=0.5)

    # AK Motor Arguments
    parser.add_argument('--ak_channel', type=str, default='COM11')
    parser.add_argument('--ak_id', type=int, default=1)
    
    # Shared
    parser.add_argument('--bitrate', type=int, default=1000000)
    parser.add_argument('--master_id', type=lambda x: int(x, 0), default=0x00)

    args = parser.parse_args()

    # # 1. Start the AK Motor Hold thread as a background daemon
    # ak_thread = threading.Thread(
    #     target=hold_ak_motor,
    #     args=(args.ak_channel, args.bitrate, args.ak_id, args.master_id),
    #     daemon=True 
    # )
    # ak_thread.start()

    # 2. Run the GL40 II control loop on the main thread
    logging.info("Press Ctrl+C to stop both motors.")
    drive_motor(
        bus_channel=args.gl_channel,
        bus_bitrate=args.bitrate,
        motor_id=args.gl_id,
        master_id=args.master_id,
        p_des=args.pos,
        v_des=args.vel
    )