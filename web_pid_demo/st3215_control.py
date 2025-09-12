from st3215 import ST3215
import time

servo = ST3215('COM4')
ids = servo.ListServos()

if ids:
    print("Connected servos:", ids)
    servo.PingServo(ids[0])
    current_position = servo.ReadPosition(ids[0])
    print("Current position:", current_position)
    
    # Sweep the motor angle from 0 to 4095 back and forth for 3 times
    for cycle in range(3):
        print(f"Starting cycle {cycle + 1}")
        
        # Sweep from 0 to 4095
        for pos in range(0, 4096, 50):
            servo.MoveTo(ids[0], pos)
            time.sleep(0.01)  # Small delay for smooth movement
        
        # Sweep back from 4095 to 0
        for pos in range(4095, -1, -50):
            servo.MoveTo(ids[0], pos)
            time.sleep(0.01)
        
        print(f"Cycle {cycle + 1} completed")
    
    moved_position = servo.ReadPosition(ids[0])
    print("Final position:", moved_position)
