import serial
import time

# Target position (0-4095 for STS 3215)
setpoint = 4095

ser = serial.Serial('COM4', 1000000, timeout=1)

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
    ser.write(cmd)
    time.sleep(0.01)  # Small delay

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
    ser.flushInput()  # Clear any pending data
    ser.write(cmd)
    time.sleep(0.01)  # Wait for response
    response = ser.read(8)  # Expected response length
    if len(response) == 8 and response[0] == 0xFF and response[1] == 0xFF:
        low_byte = response[5]
        high_byte = response[6]
        position = low_byte + (high_byte << 8)
        return position
    return None


# Read and print the original position
original_position = read_position(1)
print(f"Original position: {original_position}")

# Move to setpoint
move_motor(1, setpoint)

# Monitor position for a few seconds
print(f"Moving to setpoint: {setpoint}")
time.sleep(2)
pos = read_position(1)
if pos is not None:
    print(f"Current Position: {pos}")

# Move back to original position
print(f"Moving back to original position: {original_position}")
move_motor(1, original_position)

time.sleep(2)

# Monitor position again
pos = read_position(1)
if pos is not None:
    print(f"CurrentPosition: {pos}")

ser.close()
