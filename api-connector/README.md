# Serial Communication and Weather Data

This Python script establishes a serial connection and receives data from a serial port. It handles different types of data and performs specific actions based on the received data. Additionally, it retrieves weather data from the OpenWeatherMap API.

## Prerequisites

- Python 3.x
- Required Python packages (see `requirements.txt`)

## Installation

1. Clone the repository or download the script.

2. Install the required Python packages by running the following command:

pip install -r requirements.txt


## Usage

1. Connect your serial device to the appropriate port (e.g., COM1).

2. Run the Python script using the following command:

python climate_station.py

3. The script will continuously receive data from the serial port and perform specific actions based on the received data. It will also retrieve weather data from the OpenWeatherMap API.

## Configuration

- Serial Port: Modify the `ser = serial.Serial('COM1', 9600)` line in the script to match your serial port configuration.

- OpenWeatherMap API: If needed, modify the latitude, longitude, and app ID parameters in the script to retrieve weather data for a different location.

## License

This project is licensed under the [MIT License](LICENSE).