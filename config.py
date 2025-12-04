# config.py
import os
import pydeck as pdk
from dotenv import load_dotenv, find_dotenv

# Carrega variáveis de ambiente do arquivo .env
load_dotenv(find_dotenv(), override=True)

# Autenticação e API Keys
APP_PASSWORD = os.getenv("APP_PASSWORD", "admin")
MAPBOX_API_KEY = os.getenv("MAPBOX_API_KEY", "")
NGROK_AUTH_TOKEN = os.getenv("NGROK_AUTH_TOKEN", "")

# Configura a chave global do PyDeck se disponível
if MAPBOX_API_KEY:
    pdk.settings.mapbox_api_key = MAPBOX_API_KEY

# Caminhos
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ASSETS_DIR = os.path.join(BASE_DIR, 'assets')

# Nome do seu arquivo KML (coloque o arquivo na pasta assets/)
KML_FILENAME = "ZN_restrictions.kml" 
KML_FILE_PATH = os.path.join(ASSETS_DIR, KML_FILENAME)

# Configurações H3, 1=2562km, 5=19.7km, 8=1062m, 10=151.7m, 12=21.7m, 15=1.16m
H3_BASE_RESOLUTION = 12
H3_MIN_RESOLUTION = 5

# --- Parâmetros de Simulação (Opcional, para evitar hardcode no app) ---
# Baseado no Bitcraze’s Crazyflie 2.x nano-quadrotor
DEFAULT_ALTITUDE = 60      # m
DEFAULT_SPEED = 3          # m/s
DEFAULT_CLIMB_SPEED = 3.1  # m/s
SIMULATION_FPS = 0.05