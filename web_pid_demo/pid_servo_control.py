import serial
import time

# Example PID parameters
Kp = 1.2
Ki = 0.01
Kd = 0.05

# Target position
setpoint = 600
integral = 0
last_error = 0

ser = serial.Serial('COM3', 115200, timeout=1)

def move_motor(motor_id, position):

    # Write position command (Feetech bus servo protocol)
    cmd = bytearray([0xFF, 0xFF, motor_id, 0x05, 0x03, 0x1E, position & 0xFF, (position >> 8) & 0xFF])
    ser.write(cmd)
    time.sleep(0.1)

def read_position(motor_id):
    # Read position from encoder (Feetech bus servo protocol)
    # Instruction: 0x02 (Read), Address: 0x38 (Current Position), Length: 2
    cmd = bytearray([0xFF, 0xFF, motor_id, 0x04, 0x02, 0x38, 0x02])
    ser.write(cmd)
    time.sleep(0.05)
    response = ser.read(8)  # Response length may vary
    if len(response) >= 8:
        pos_l = response[5]
        pos_h = response[6]
        position = pos_l + (pos_h << 8)
        return position
    return None

for i in range(100):
    # Read current position from motor encoder
    current_position = read_position(1)
    if current_position is None:
        print("Failed to read position")
        continue
    error = setpoint - current_position
    integral += error
    derivative = error - last_error
    output = Kp * error + Ki * integral + Kd * derivative
    position = int(current_position + output)
    move_motor(1, max(0, min(1023, position)))
    last_error = error
    time.sleep(0.05)

ser.close()
