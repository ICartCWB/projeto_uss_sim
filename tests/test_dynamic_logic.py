import sys
import os
from datetime import datetime, timedelta
import h3
import numpy as np
from geopy.distance import geodesic

# --- Configuração de Path e Imports ---
# Permite importar módulos da pasta 'core'
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.obstacle_model import H3ObstacleModel
from core.models import FlightIntent, Waypoint
# Assumimos que geo_algorithms foi importado e está disponível (ou definimos o mock localmente)
from core.geo_algorithms import generate_fine_res_cells 
import config # Para pegar as resoluções

# --- Setup Global (Necessário para criar intents de forma consistente) ---

# Mock da função de criação de intenção (similar ao test_integration, mas focado no 4D)
def create_mock_intent(start_coords, end_coords, flight_id, alt_m, start_time, duration_sec=100, speed=10.0):
    # A Duração Total (ETA) deve ser calculada dentro do register_flight, 
    # mas a Intent precisa de tempo para calcular a velocidade.
    
    cells = generate_fine_res_cells(
        line_vector=[start_coords, end_coords], 
        base_resolution=config.H3_BASE_RESOLUTION
    )
    
    wp_start = Waypoint(lat=start_coords[0], lon=start_coords[1], alt=alt_m, timestamp=start_time)
    wp_end = Waypoint(lat=end_coords[0], lon=end_coords[1], alt=alt_m, timestamp=start_time + timedelta(seconds=duration_sec)) 
    
    return FlightIntent(
        operator_id="Tester",
        flight_id=flight_id,
        waypoints=[wp_start, wp_end],
        start_time=start_time,
        intent_cells=cells,
        speed_m_s=speed,
        ascent_duration_sec=10,
        descent_duration_sec=10
    )

# --- FUNÇÃO DE TESTE PRINCIPAL ---

def test_dynamic_model_logic():
    print("=========================================================")
    print("🧪 TESTES DE LÓGICA DINÂMICA (get_ratio & check_dynamic)")
    print("=========================================================")

    # 0. SETUP: Criar Modelo Limpo e Voo Base
    
    # Modelo sem obstáculos estáticos (KML vazio)
    model = H3ObstacleModel(
        shapely_polygons=[], 
        base_res=config.H3_BASE_RESOLUTION, 
        min_res=config.H3_MIN_RESOLUTION
    )
    
    T0 = datetime.now() + timedelta(seconds=5) # Início em 5 segundos
    ALT_TEST_A = 120.0 # Nível 4
    
    # Voo A: Base para o teste, atravessa uma área pequena no centro
    start_a = [-23.200, -45.880]
    end_a = [-23.205, -45.885]
    duration_sec = 20 # 20s de cruzeiro
    contingency_sec = 60 # 1 min de contingência

    voo_a = create_mock_intent(
        start_a, end_a, 
        flight_id="FLT_A_BASE", 
        alt_m=ALT_TEST_A, 
        start_time=T0,
        duration_sec=duration_sec
    )
    
    # -------------------------------------------------------------
    # REGISTRO - A única fonte de obstáculo dinâmico
    # -------------------------------------------------------------
    model.register_flight(voo_a, contingency_time_sec=contingency_sec)
    
    # Célula central para checagem (uma célula no meio do caminho)
    cell_check_base = h3.latlng_to_cell(-23.203, -45.883, config.H3_BASE_RESOLUTION)
    cell_check_parent = h3.cell_to_parent(cell_check_base, config.H3_MIN_RESOLUTION)

    # Cálculo da Janela de Bloqueio TOTAL (para referência)
    cruise_time_total = geodesic(start_a, end_a).meters / voo_a.speed_m_s
    block_end_time = T0 + timedelta(seconds=voo_a.ascent_duration_sec + cruise_time_total + voo_a.descent_duration_sec + contingency_sec)
    print(f"\n[INFO] Voo A Bloqueado de {T0.strftime('%H:%M:%S')} até {block_end_time.strftime('%H:%M:%S')}")
    print(f"[INFO] Célula Base de Checagem: {cell_check_base}")

    
    # =============================================================
    # CENÁRIO T1: CONFLITO DINÂMICO (TEMPO E ALTURA OK)
    # =============================================================
    
    T_check_T1 = T0 + timedelta(seconds=29) # 30s após o início (dentro da janela)
    print("\n--- TESTE T1: Conflito Total 4D (Esperado: 2.0 / FLT_A_BASE) ---")
    
    # 1. get_ratio (deve retornar 2.0)
    ratio_t1 = model.get_ratio(cell_check_base, model.base_resolution, ALT_TEST_A, T_check_T1)
    
    # 2. check_dynamic_occupation (deve retornar o ID)
    id_t1 = model.check_dynamic_occupation(cell_check_base, model._get_altitude_level(ALT_TEST_A), T_check_T1)
    
    assert ratio_t1 == 2.0, f"T1 Falha get_ratio: Esperava 2.0, obteve {ratio_t1}"
    assert id_t1 == "FLT_A_BASE", f"T1 Falha check_dynamic: Esperava FLT_A_BASE, obteve {id_t1}"
    print("✅ T1 PASSOU (Restrição 4D na Base Res)")


    # =============================================================
    # CENÁRIO T2: CONFLITO DE ALTITUDE (NÍVEIS DIFERENTES)
    # =============================================================
    
    ALT_TEST_B = 60.0 # Nível 2 (Diferente de 120.0 = Nível 4)
    T_check_T2 = T_check_T1 # Mesmo tempo

    print("\n--- TESTE T2: Conflito de Altitude (Esperado: 0.0 / None) ---")
    
    ratio_t2 = model.get_ratio(cell_check_base, model.base_resolution, ALT_TEST_B, T_check_T2)
    id_t2 = model.check_dynamic_occupation(cell_check_base, model._get_altitude_level(ALT_TEST_B), T_check_T2)

    assert ratio_t2 == 0.0, f"T2 Falha get_ratio: Esperava 0.0, obteve {ratio_t2}"
    assert id_t2 is None, f"T2 Falha check_dynamic: Esperava None, obteve {id_t2}"
    print("✅ T2 PASSOU (Ignorou nível de altitude diferente)")


    # =============================================================
    # CENÁRIO T3: CONFLITO DE TEMPO (MUITO TARDE)
    # =============================================================
    
    # Checagem DEPOIS que a janela de bloqueio termina
    T_check_T3 = block_end_time + timedelta(minutes=5)
    
    print("\n--- TESTE T3: Fora da Janela Temporal (Esperado: 0.0 / None) ---")
    
    ratio_t3 = model.get_ratio(cell_check_base, model.base_resolution, ALT_TEST_A, T_check_T3)
    id_t3 = model.check_dynamic_occupation(cell_check_base, model._get_altitude_level(ALT_TEST_A), T_check_T3)

    assert ratio_t3 == 0.0, f"T3 Falha get_ratio: Esperava 0.0, obteve {ratio_t3}"
    assert id_t3 is None, f"T3 Falha check_dynamic: Esperava None, obteve {id_t3}"
    print("✅ T3 PASSOU (Janela de tempo expirada)")


    # =============================================================
    # CENÁRIO T4: PROPAGAÇÃO PARA PAIS (VERIFICAÇÃO DE ZOOM-OUT)
    # =============================================================
    
    T_check_T4 = T_check_T1 # Mesmo tempo (Dentro da janela)

    print("\n--- TESTE T4: Checagem em Célula PAI (Esperado: Ratio < 1.0 / FLT_A_BASE) ---")
    
    # Checa a célula PAI (Resolução Mínima)
    ratio_t4 = model.get_ratio(cell_check_parent, model.min_resolution, ALT_TEST_A, T_check_T4)
    
    # Checa o ID apenas para confirmação (usa check_dynamic_occupation)
    id_t4_check = model.check_dynamic_occupation(cell_check_parent, model._get_altitude_level(ALT_TEST_A), T_check_T4)
    
    # O Ratio deve ser maior que zero (para forçar o zoom-in) e menor que 1.0.
    assert ratio_t4 > 0.0, f"T4 Falha get_ratio: Esperava Ratio > 0.0 no PAI, obteve {ratio_t4}"
    assert ratio_t4 < 1.0, f"T4 Falha get_ratio: Ratio {ratio_t4} é 1.0 ou mais (esperava parcial)"
    assert id_t4_check == "FLT_A_BASE", f"T4 Falha no ID do PAI: Esperava FLT_A_BASE, obteve {id_t4_check}"
    
    print(f"✅ T4 PASSOU (Propagação Dinâmica para Pais Funcional. Ratio Parcial: {ratio_t4:.4f})")

    
    # =============================================================
    # CENÁRIO T5: LIMPEZA (DEREGISTER)
    # =============================================================
    
    print("\n--- TESTE T5: Remoção (Deregister) ---")
    
    # Remove o voo
    model.deregister_flight("FLT_A_BASE")
    
    # Tenta checar a célula base novamente
    ratio_t5 = model.get_ratio(cell_check_base, model.base_resolution, ALT_TEST_A, T_check_T1)
    id_t5 = model.check_dynamic_occupation(cell_check_base, model._get_altitude_level(ALT_TEST_A), T_check_T1)

    assert ratio_t5 == 0.0, f"T5 Falha get_ratio: Esperava 0.0 após remoção, obteve {ratio_t5}"
    assert id_t5 is None, f"T5 Falha check_dynamic: Esperava None após remoção, obteve {id_t5}"
    print("✅ T5 PASSOU (Voo removido com sucesso)")
    
    print("\n=========================================================")
    print("✨ TESTES DINÂMICOS CONCLUÍDOS COM SUCESSO!")
    print("=========================================================")


if __name__ == "__main__":
    test_dynamic_model_logic()