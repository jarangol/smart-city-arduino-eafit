from dotenv import load_dotenv
import random
import os
import requests

# Cargar las variables de entorno desde el archivo .env
load_dotenv()

def get_weather():
	print("ejecutando")
	try:
		cities = [
			{"name": "El Poblado", "lat": 6.2, "lon": -75.5787},
			{"name": "Sabaneta", "lat": 6.1523, "lon": -75.6168},
			{"name": "Bello", "lat": 6.3378, "lon": -75.5546},
			{"name": "Rionegro", "lat": 6.15, "lon": -75.3776}
		]

		# Obtener ciudad
		city = random.choice(cities)

		api_key = os.getenv('WEATHER_API_KEY')

		# llamado a API
		url = 'https://api.openweathermap.org/data/2.5/weather'
		params = {
			'lat': city['lat'],
			'lon': city['lon'],
			'appid': api_key,
			'units': 'metric'
		}
	
		print("getting weather information: ", city['name'])
		response = requests.get(url, params=params)
		response.raise_for_status()  # Check for any request errors
		weather_data = response.json()  # Process weather data as needed
		message = "{}: {} °C".format(city['name'], weather_data['main']['temp'])
		return message
	except requests.exceptions.RequestException as e:
		print("Failed to retrieve weather data:", e)
