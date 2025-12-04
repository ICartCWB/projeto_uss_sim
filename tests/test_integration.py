import sys
import os
import numpy as np
from datetime import datetime, timedelta
import h3

# --- Configuração de Path para importar o módulo 'core' ---
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import config
from core.obstacle_model import build_obstacle_model_from_kml
from core.hcp import check_route_feasibility
from core.models import FlightIntent, Waypoint
from core.geo_algorithms import generate_fine_res_cells

# --- Classes Mock/Dummy para facilitar o teste sem rodar o app inteiro ---
def create_mock_intent(start_coords, end_coords, flight_id="test_flight", alt_m=50, start_time=None):
    if start_time is None:
        start_time = datetime.now()
    
    # 1. Gerar Células da Rota (Fundamental para a nova lógica)
    # Usa a resolução base definida no config
    cells = generate_fine_res_cells(
        line_vector=[start_coords, end_coords], 
        base_resolution=config.H3_BASE_RESOLUTION
    )
    
    if not cells:
        print(f"ERRO: Falha ao gerar células para o voo {flight_id}")
        cells = []

    # 2. Criar Waypoints (Apenas Início e Fim são necessários agora)
    wp_start = Waypoint(lat=start_coords[0], lon=start_coords[1], alt=alt_m, timestamp=start_time)
    wp_end = Waypoint(lat=end_coords[0], lon=end_coords[1], alt=alt_m, timestamp=start_time) 
    
    return FlightIntent(
        operator_id="Tester",
        flight_id=flight_id,
        waypoints=[wp_start, wp_end],
        start_time=start_time,
        intent_cells=cells,         # <--- CAMPO NOVO CRÍTICO
        speed_m_s=15.0,             # Velocidade mais realista
        ascent_duration_sec=10,
        descent_duration_sec=10
    )

def run_tests():
    print("===========================================================")
    print("🚁 INICIANDO TESTES DE INTEGRAÇÃO: MODELO DE OBSTÁCULOS & NAV")
    print("===========================================================")

    # 1. Carregar Modelo
    print(f"\n[SETUP] Carregando KML de: {config.KML_FILE_PATH}")
    try:
        # Tenta carregar o KML real. Se não existir, cria um modelo vazio para não quebrar o script.
        if os.path.exists(config.KML_FILE_PATH):
            obstacle_model = build_obstacle_model_from_kml(
                config.KML_FILE_PATH, 
                base_res=config.H3_BASE_RESOLUTION, 
                min_res=config.H3_MIN_RESOLUTION
            )
            print(f"[SETUP] Sucesso. {len(obstacle_model.obstacle_geometries)} polígonos carregados.")
        else:
            print("[ERRO] Arquivo KML não encontrado em assets/. Rodando com mapa vazio (Testes de obstáculo falharão).")
            return
    except Exception as e:
        print(f"[ERRO CRÍTICO] Falha ao ler KML: {e}")
        return

    # --- CENÁRIOS DE TESTE ---

    scenarios = [
        {
            "name": "CENÁRIO 1: Rota Livre (SJC)",
            "start": [-23.192, -45.887],
            "end": [-23.210, -45.875],
            "expect_success": True
        },
        {
            "name": "CENÁRIO 2: Rota c/ Obstáculo (SP - Centro/Norte)",
            # Coordenadas que você passou que devem cruzar uma NFZ
            "start": [-23.484, -46.696], 
            "end": [-23.553, -46.612],
            "expect_success": False
        },
        {
            "name": "CENÁRIO 3: Rota Difícil (Corredor Estreito)",
            # Coordenadas próximas a obstáculos
            "start": [-23.514, -46.589],
            "end": [-23.545, -46.602],
            "expect_success": True # Assumindo que passa raspando mas passa
        },
        {
            "name": "CENÁRIO 4: Rota c/ Obstáculo (SP - Centro/Norte)",
            # Coordenadas que você passou que devem cruzar uma NFZ
            "start": [-23.514, -46.589], 
            "end": [-23.553, -46.612],
            "expect_success": False
        },
        {
            "name": "CENÁRIO 5: Rota Difícil (Corredor Estreito)",
            # Coordenadas próximas a obstáculos
            "start": [-23.5420, -46.6719],
            "end": [-23.5397, -46.6605],
            "expect_success": True # Assumindo que passa raspando mas passa
        }
    ]

    for sc in scenarios:
        print(f"\n--- Executando {sc['name']} ---")
        print(f"Rota: {sc['start']} -> {sc['end']}")
        
        intent = create_mock_intent(sc['start'], sc['end'])
        
        # Chama o TacticalNavigator
        result = check_route_feasibility(
            intent=intent,
            obstacle_model=obstacle_model
        )
        
        success = result['success']
        print(f"Resultado: {'APROVADO' if success else 'BLOQUEADO'}")
        
        if not success:
            print(f"Motivo: {result.get('reason')} | Célula: {result.get('cell_id')}")

        # Verificação do teste
        if success == sc['expect_success']:
            print(">> ✅ TESTE PASSOU (Comportamento esperado)")
        else:
            print(f">> ❌ TESTE FALHOU (Esperava {sc['expect_success']}, obteve {success})")
            # Se falhar no Cenário 2 e você não tiver o KML carregado certo, é normal.

    
    # --- CENÁRIO 4: TESTE DINÂMICO 4D ---
    print(f"\n--- Executando CENÁRIO 6: Conflito Dinâmico (Drone vs Drone) ---")
    
    # Data/Hora base para o teste (garante sincronia)
    now = datetime.now()

    # 1. Criar e Registrar um Voo (Voo A)
    print("1. Criando e Registrando Voo A (Obstáculo Dinâmico)...")
    
    # Voo A: Rota Leste -> Oeste em SJC
    voo_a = create_mock_intent(
        start_coords=[-23.200, -45.870], 
        end_coords=[-23.200, -45.890], 
        flight_id="flight_A",
        start_time=now
    )
    
    # Registra no modelo (Isso preenche o dynamic_grid com o bloqueio total de tempo)
    # O tempo de bloqueio será: now até (now + tempo_voo + contingencia)
    obstacle_model.register_flight(voo_a, contingency_time_sec=30)
    
    # 2. Tentar criar Voo B que cruza a rota do Voo A no mesmo intervalo
    print("2. Tentando Validar Voo B (Rota Cruzada no Mesmo Horário)...")
    
    # Voo B: Rota Norte -> Sul (Cruzando o Voo A no meio)
    voo_b = create_mock_intent(
        start_coords=[-23.190, -45.880], 
        end_coords=[-23.210, -45.880], 
        flight_id="flight_B",
        start_time=now # Mesmo horário de partida
    )
    
    # Verificação de Viabilidade
    # O check deve consultar o obstacle_model e ver as células ocupadas pelo Voo A
    result_b = check_route_feasibility(
        intent=voo_b,
        obstacle_model=obstacle_model
    )
    
    # 3. Validação do Resultado
    if not result_b['success'] and result_b.get('reason') == 'DYNAMIC_CONFLICT':
        print(f"Resultado: BLOQUEADO CORRETAMENTE")
        print(f"Motivo: {result_b['reason']}")
        print(f"Conflito com: {result_b.get('obstacle_id')} na célula {result_b.get('cell_id')}")
        print(">> ✅ TESTE PASSOU (Conflito Temporal Detectado)")
    elif result_b['success']:
        print(f"Resultado:  APROVADO (ERRO - Deveria ter bloqueado)")
        print(">> ❌ TESTE FALHOU (Colisão não detectada)")
    else:
        print(f"Resultado:  BLOQUEADO POR OUTRO MOTIVO: {result_b.get('reason')}")
        print(">> ❌ TESTE INCONCLUSIVO")

if __name__ == "__main__":
    run_tests()