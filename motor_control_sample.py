import serial
import time

def move_motor(motor_id, position):
    """
    Control Feetech STS 3215 motor via serial.
    motor_id: int (1-254)
    position: int (0-1023)
    """
    # Example command format (adjust for your motor's protocol)
    cmd = bytearray([0xFF, 0xFF, motor_id, 0x05, 0x03, 0x1E, position & 0xFF, (position >> 8) & 0xFF])
    with serial.Serial('COM3', 1000000, timeout=1) as ser:
        ser.write(cmd)
        time.sleep(0.1)

if __name__ == "__main__":
    # Move motor 1 to position 512 (middle)
    move_motor(1, 512)
