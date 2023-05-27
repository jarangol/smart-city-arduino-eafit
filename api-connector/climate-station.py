import serial
from tweet import post_tuit
from weather import get_weather
from datetime import datetime

# Establish serial connection
# Replace 'COM1' with the appropriate serial port
ser = serial.Serial('/dev/tty.usbmodem14101', 9600)

# Loop to continuously receive data
while True:
    # Read data from the serial port and decode it
    data = ser.readline().decode().strip()

    if data == 'a':  # High traffic on Street A
        print("High traffic on Street A")
        post_tuit(f"{dt_string} High traffic on Street A")
    elif data == 't':  # High traffic in tunnel
        print("High traffic in tunnel")
        now = datetime.now()
        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
        post_tuit(f"{dt_string} High traffic in tunnel")
    elif data == 'w':  # Get weather
        print("Getting weather data...")
        message = get_weather()
        print(message)
        ser.write(message.encode())
    else:
        # Invalid data received
        print(data)

# Close the serial connection (unreachable in this infinite loop)
ser.close()
