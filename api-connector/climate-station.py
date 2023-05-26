import serial
import requests
from tweet import post_tuit

# Establish serial connection
# Replace 'COM1' with the appropriate serial port
ser = serial.Serial('/dev/tty.usbmodem14101', 9600)

# Loop to continuously receive data
while True:
    # Read data from the serial port and decode it
    data = ser.readline().decode().strip()

    if data == 'a':  # High traffic on Street A
        print("High traffic on Street A")
        post_tuit("High traffic on Street A")
    elif data == 't':  # High traffic in tunnel
        print("High traffic in tunnel")
        post_tuit("High traffic in tunnel")
    elif data == 'w':  # Get weather
        print("Getting weather data...")

        # Call OpenWeatherMap API
        url = 'https://api.openweathermap.org/data/2.5/weather'
        params = {
            'lat': '51.509865',
            'lon': '-0.118092',
            'appid': '35033edc5bae430e28bc567c9d680056',
            'units': 'metric'
        }

        try:
            response = requests.get(url, params=params)
            response.raise_for_status()  # Check for any request errors
            weather_data = response.json()  # Process weather data as needed
            print("Weather data:", weather_data)
        except requests.exceptions.RequestException as e:
            print("Failed to retrieve weather data:", e)
    else:
        # Invalid data received
        print("Invalid data received:", data)

# Close the serial connection (unreachable in this infinite loop)
ser.close()
