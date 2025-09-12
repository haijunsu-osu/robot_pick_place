import serial
import time

def read_position(motor_id):
    """
    Read current position of Feetech STS 3215 motor.
    motor_id: int (1-254)
    Returns: int (0-4095) or None if error
    """
    length = 0x04
    instruction = 0x02  # Read
    address = 0x38  # Present position
    data_length = 0x02
    checksum = (~(motor_id + length + instruction + address + data_length)) & 0xFF
    cmd = bytearray([0xFF, 0xFF, motor_id, length, instruction, address, data_length, checksum])
    with serial.Serial('COM4', 1000000, timeout=1) as ser:
        ser.write(cmd)
        response = ser.read(8)  # Expected response length
        if len(response) == 8 and response[0] == 0xFF and response[1] == 0xFF:
            low_byte = response[5]
            high_byte = response[6]
            position = low_byte + (high_byte << 8)
            return position
    return None

def move_motor(motor_id, position):
    """
    Control Feetech STS 3215 motor via serial.
    motor_id: int (1-254)
    position: int (0-4095)
    """
    # STS 3215 protocol: Write goal position (address 0x2A)
    low_byte = position & 0xFF
    high_byte = (position >> 8) & 0xFF
    length = 0x05
    instruction = 0x03
    address = 0x2A
    checksum = (~(motor_id + length + instruction + address + low_byte + high_byte)) & 0xFF
    cmd = bytearray([0xFF, 0xFF, motor_id, length, instruction, address, low_byte, high_byte, checksum])
    with serial.Serial('COM4', 1000000, timeout=1) as ser:
        ser.write(cmd)
        time.sleep(0.01)  # Small delay

def sweep_motor(motor_id, start_pos, end_pos, step=50):
    """
    Sweep motor from start_pos to end_pos in steps.
    """
    if start_pos < end_pos:
        positions = range(start_pos, end_pos + 1, step)
    else:
        positions = range(start_pos, end_pos - 1, -step)
    for pos in positions:
        move_motor(motor_id, pos)

if __name__ == "__main__":
    motor_id = 1
    
    # (1) Read the current position
    current_pos = read_position(motor_id)
    print(f"Initial position: {current_pos}")
    
    # (2) Sweep the motor from 0 to 4095 and back 3 times
    for cycle in range(3):
        print(f"Starting cycle {cycle + 1}")
        sweep_motor(motor_id, 0, 4095)  # 0 to 4095
        sweep_motor(motor_id, 4095, 0)  # 4095 to 0
        print(f"Cycle {cycle + 1} completed")
    
    # (3) Read the current position again
    final_pos = read_position(motor_id)
    print(f"Final position: {final_pos}")
