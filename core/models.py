from dataclasses import dataclass, field
from typing import List, Optional
from datetime import datetime

@dataclass
class Waypoint:
    lat: float
    lon: float
    alt: float
    timestamp: datetime # Mantido, mas o controle principal será pelo start_time do Intent

@dataclass
class FlightIntent:
    operator_id: str
    flight_id: str
    waypoints: List[Waypoint] # Mantemos Início e Fim
    start_time: datetime
    
    # --- Novos Atributos ---
    intent_cells: List[str] = field(default_factory=list) # Células na resolução base
    speed_m_s: float = 3.0 # Velocidade média de cruzeiro (Ref: PyBullet Drone padrão)
    
    # Tempos de operação (fixos ou estimados)
    ascent_duration_sec: int = 10   # Tempo para subir até a altitude de cruzeiro
    descent_duration_sec: int = 10  # Tempo para descer ao solo
    
    status: str = "PENDING"
    version: str = field(default_factory=lambda: datetime.now().isoformat())

    @property
    def origin(self):
        return self.waypoints[0]

    @property
    def destination(self):
        return self.waypoints[-1]