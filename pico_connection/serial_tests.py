import serial
import time

serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

last_time = None
try:
    while True:
        if serial_port.in_waiting > 0:
            current_time = time.time()
            line = serial_port.readline().decode('utf-8').strip()
            if last_time is not None:
                delta = current_time - last_time
                print(f'Czas między wiadomościami: {delta} sekund')
            last_time = current_time
except KeyboardInterrupt:
    print("Zakończono monitorowanie")
    serial_port.close()
