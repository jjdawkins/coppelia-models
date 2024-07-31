import serial

# Configure the serial connections (replace '/dev/ttyAMA0' with your device)
ser = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=1)

try:
    while True:
        # Read data from serial port
        if ser.in_waiting > 0:
            data = ser.readline()
            print(f'Received data: {data}')

except KeyboardInterrupt:
    print("Keyboard Interrupt")

finally:
    # Clean up
    ser.close()