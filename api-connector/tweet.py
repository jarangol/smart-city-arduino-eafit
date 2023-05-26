import tweepy
from dotenv import load_dotenv
import os

# Cargar las variables de entorno desde el archivo .env
load_dotenv()


def post_tuit(message):

    # 4 cadenas para la autenticacion
    token = os.getenv('TWITTER_BEARER_TOKEN')
    consumer_key = os.getenv('CONSUMER_KEY')
    consumer_secret = os.getenv('CONSUMER_SECRET')
    access_token = os.getenv('ACCESS_TOKEN')
    access_token_secret = os.getenv('ACCESS_TOKEN_SECRET')

    api = tweepy.Client(bearer_token=token,
                        access_token=access_token,
                        access_token_secret=access_token_secret,
                        consumer_key=consumer_key,
                        consumer_secret=consumer_secret)

    api.create_tweet(text=message)


# txtTuit = input("Ingresa el texto del tuit: ")
# post_tuit(txtTuit)
