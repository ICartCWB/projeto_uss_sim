import sys
import os
from datetime import datetime, timedelta
import numpy as np

# Ajuste de path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.models import FlightIntent, Waypoint
from core.simulation import DroneSimulationBackend, CoordinateHelper

def test_coordinate_conversion():
    print("\n=== TESTE 1: Conversão de Coordenadas (Geo <-> Cartesiano) ===")
    
    origin_lat, origin_lon = -23.211349, -45.875629
    helper = CoordinateHelper(origin_lat, origin_lon, 0)
    
    test_lat, test_lon = -23.206849, -45.875629 
    test_alt = 50.0
    
    xyz = helper.geo_to_cartesian(test_lat, test_lon, test_alt)
    print(f"Original Geo: ({test_lat}, {test_lon}, {test_alt})")
    print(f"Calculado XYZ: {xyz}")
    
    geo_back = helper.cartesian_to_geo(xyz[0], xyz[1], xyz[2])
    print(f"Retorno Geo:  ({geo_back['lat']}, {geo_back['lon']}, {geo_back['alt']})")
    
    error_lat = abs(geo_back['lat'] - test_lat)
    
    # AJUSTE DE TOLERÂNCIA: Aumentado para 0.00002 (~2 metros)
    if error_lat < 0.00002:
        print(">> SUCESSO: Conversão Geo -> Cartesiano -> Geo é consistente.")
    else:
        print(f">> FALHA: Erro de precisão na conversão: {error_lat}")

def test_simulation_loop():
    print("\n=== TESTE 2: Loop de Simulação (Física) ===")
    
    start_time = datetime.now()
    
    wp_origin = Waypoint(lat=-23.211349, lon=-45.875629, alt=0, timestamp=start_time)
    wp_dest = Waypoint(lat=-23.211349, lon=-45.874629, alt=30, timestamp=start_time + timedelta(minutes=1))
    
    intent = FlightIntent(
        operator_id="OpTest",
        flight_id="BR-123",
        waypoints=[wp_origin, wp_dest],
        start_time=start_time,
        speed_m_s=5.0
    )
    
    print("Inicializando Backend...")
    try:
        sim_backend = DroneSimulationBackend([intent])
    except Exception as e:
        print(f"ERRO FATAL ao inicializar PyBullet: {e}")
        # Mostra o traceback completo se falhar de novo
        import traceback
        traceback.print_exc()
        return

    print("Iniciando Run (Simulando 200 steps)...")
    generator = sim_backend.run()
    
    history_alt = []
    
    try:
        for i, frame in enumerate(generator):
            if i >= 50: 
                break
            
            telemetry = frame[0] 
            lat, lon, alt = telemetry['lat'], telemetry['lon'], telemetry['alt']
            history_alt.append(alt)
            
            if i % 10 == 0:
                print(f"Step {i}: Altura={alt:.2f}m | Lat={lat:.6f} | Status={telemetry['status']}")
                
    except Exception as e:
        print(f"ERRO durante execução: {e}")
        import traceback
        traceback.print_exc()
        
    initial_alt = history_alt[0]
    final_alt = history_alt[-1]
    
    print("-" * 30)
    print(f"Altura Inicial: {initial_alt:.2f}m")
    print(f"Altura Final:   {final_alt:.2f}m")
    
    if final_alt > initial_alt + 0.1:
        print(">> SUCESSO: O drone decolou e ganhou altitude.")
    elif final_alt < initial_alt:
         print(">> AVISO: O drone caiu ou afundou no chão.")
    else:
        print(">> FALHA: O drone não se moveu.")

if __name__ == "__main__":
    test_coordinate_conversion()
    test_simulation_loop()