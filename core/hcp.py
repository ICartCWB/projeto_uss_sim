# Hierarchical Conflict Probe (HCP) with H3 DGGS

import h3
import numpy as np
import datetime
from geopy.distance import geodesic
from typing import List, Tuple, Dict, Any, Optional
from datetime import timedelta

from core.geo_algorithms import find_next_step_on_vector

# Importa o modelo de obstáculos definido anteriormente
from core.obstacle_model import H3ObstacleModel
from core.models import FlightIntent

# =============================================================================
# 1. FUNÇÃO PRINCIPAL DE VALIDAÇÃO 4D (Strategic Deconfliction)
# =============================================================================

def check_route_feasibility(
    intent: FlightIntent,
    obstacle_model: H3ObstacleModel,
    time_buffer_sec: int = 60
) -> Dict[str, Any]:
    """
    Orquestra a navegação tática ao longo do vetor de voo proposto para validar 
    a viabilidade 4D (Espaço e Tempo).
    
    Diferente da versão original que buscava um caminho, esta função VERIFICA 
    se o caminho direto proposto está livre de:
    1. Obstáculos Estáticos (via obstacle_model)
    2. Obstáculos Dinâmicos (outras rotas) respeitando a janela de tempo.

    Lógica de Tempo:
    - Calcula o tempo estimado de chegada (ETA) em cada célula H3 do trajeto baseada na velocidade.
    - Verifica se alguma outra rota ocupa aquela célula no intervalo [ETA - buffer, ETA + buffer].

    Args:
        intent (FlightIntent): O objeto da intenção de voo contendo:
                               - start_point (lat, lon, alt)
                               - end_point (lat, lon, alt)
                               - start_time (datetime)
                               - speed_m_s (float) ou duration (para cálculo 4D)
        obstacle_model (H3ObstacleModel): O modelo contendo zonas NFZ estáticas.
        active_intents (List[FlightIntent]): Lista de intenções JÁ APROVADAS que representam 
                                             obstáculos dinâmicos.
        time_buffer_sec (int): Janela de segurança temporal (padrão 1 min). Se a rota passa
                               pela célula X às 10:00:00, a célula é considerada ocupada 
                               de 09:59:00 até 10:01:00 para outros drones.

    Returns:
        dict: Dicionário indicando sucesso ou detalhando o conflito.
        
        Exemplos de Retorno:
        - Sucesso:
          {'success': True, 'message': 'Rota livre 4D'}
          
        - Conflito Estático:
          {'success': False, 'reason': 'STATIC_OBSTACLE', 'cell_id': '89...', 'obstacle_id': 12}
          
        - Conflito Dinâmico (Outro Drone):
          {'success': False, 'reason': 'DYNAMIC_CONFLICT', 'conflicting_flight_id': 'fly_02', 'time_overlap': '10:05:30'}
    """

    # Extração dos dados da intenção
    start_point = np.array((intent.waypoints[0].lat, intent.waypoints[0].lon))
    end_point = np.array((intent.waypoints[-1].lat, intent.waypoints[-1].lon))

    # Definição da altitude discreta (1, 2, 3, 4)
    # Mapeia a altitude em metros para o nível correspondente
    alt_level = 4 # Default
    if intent.waypoints[0].alt > 105: alt_level = 4
    elif intent.waypoints[0].alt > 75: alt_level = 3
    elif intent.waypoints[0].alt > 45: alt_level = 2
    elif intent.waypoints[0].alt > 15: alt_level = 1

    # Extração do tempo
    start_time = intent.waypoints[0].timestamp
    # Defina uma velocidade média ou pegue do intent se disponível
    speed_m_s = intent.speed_m_s if hasattr(intent, 'speed_m_s') else 3.0 

    # --- 1. Initial State and Setup ---
    start_point = np.array(start_point)
    end_point = np.array(end_point)
    line_vector = (start_point, end_point)
    current_point = start_point
    # Use the model's defined min_resolution and base_resolution
    current_res = obstacle_model.min_resolution
    BASE_RESOLUTION = obstacle_model.base_resolution
    MIN_RESOLUTION = obstacle_model.min_resolution
    max_iterations = 100 # Loop stop criteria in a case of error

    def _check_dynamic_conflict(cell_h3: str, time_arrival: datetime, alt_level: int) -> Optional[str]:
        """
        Verifica se a célula está ocupada no nível de altura específico 
        dentro da janela de tempo [T_arrival - buffer, T_arrival + buffer].
        Retorna o ID do conflito ou None.
        """
        # A lógica real delegaria para o obstacle_model ampliado:
        # return obstacle_model.is_occupied_4d(cell_h3, alt_level, time_arrival, time_buffer_sec)
        
        return obstacle_model.check_dynamic_occupation(cell_h3, alt_level, time_arrival, time_buffer_sec)

    iteration_count = 0
    previous_point = np.empty(2)
    dynamic_conflict_id = None

    # --- Main Decision Logic ---
    while True:
        # Safety break
        iteration_count += 1
        if iteration_count > max_iterations:
            return {
                'success': False,
                'reason': f'Navigation exceeded max iterations ({max_iterations}).',
                'obstacle_id': None,
                'cell_id': h3.latlng_to_cell(current_point[0], current_point[1], current_res)
            }

        # --- Step 1: Check current location ---
        current_cell = h3.latlng_to_cell(current_point[0], current_point[1], current_res)
        target_cell = h3.latlng_to_cell(end_point[0], end_point[1], current_res)

        # a. Cálculo do Tempo Estimado de Chegada (ETA) nesta célula
        dist_flown_m = geodesic(tuple(start_point), tuple(current_point)).meters
        time_elapsed_sec = dist_flown_m / speed_m_s
        current_eta = start_time + timedelta(seconds=time_elapsed_sec)

        # b. Verificação de Obstáculo Estático (NFZ) - Altura não importa tanto aqui se NFZ for do chão ao teto
        # Mas se NFZ tiver altura, você pode passar alt_level também.
        restriction_ratio = obstacle_model.get_ratio(
            current_cell, 
            current_res, 
            alt_level=alt_level, 
            check_time=current_eta
        )
        
        if restriction_ratio > 0:
            # Area of potential conflict found at the current location

            if current_res < BASE_RESOLUTION:
                print("ZOOM-IN Iteração:", iteration_count, "Resolução:", current_res, "Célula:", current_cell)
                # Zoom-in for more precision
                current_res = min(current_res + 1, BASE_RESOLUTION)
                current_cell = h3.latlng_to_cell(current_point[0], current_point[1], current_res)

                # Re-evaluate at the new resolution without advancing the point
                if np.array_equal(current_point, start_point):
                    # The start point don't need the backward logic
                    continue
                # This logic corrects the zoom-in to the previous cell.
                reversed_line_vector = (end_point, start_point)
                # Using the current one as a starting point for the reverse search
                previous_base_cell, previous_point = find_next_step_on_vector(
                    current_point,
                    current_cell,
                    reversed_line_vector,
                    current_res
                )
                # TODO: implementar verificação de pai da célula aproximada para não ficar em loop infinito
                # current_cell_coords = tuple(current_point.tolist())
                # end_point_coords = tuple(end_point.tolist())
                # print(current_point,current_cell,reversed_line_vector,current_res,tolerance_factor)
                current_point = previous_point
                continue

            elif restriction_ratio >= 1 and current_res == BASE_RESOLUTION:
                # Verificação de Conflito Dinâmico (Outros Drones)
                dynamic_conflict_id = _check_dynamic_conflict(current_cell, current_eta, alt_level)

                # Conflict found at the base resolution.

                restriction_ratio_previous = obstacle_model.get_ratio(previous_base_cell, current_res, alt_level, current_eta)

                if restriction_ratio_previous >= 1:
                    # The static conflict might have started before this cell.
                    print("Viu conflito e retornou na célula:",current_cell)
                    previous_base_cell = current_cell
                    continue
                else:
                    # If the previous cell was clear, then this is the first point of collision.
                    obstacle_id = obstacle_model.base_obstacle_cells.get(current_cell)

                                        # Tratamento de retorno
                    if dynamic_conflict_id:
                        print(f"Conflito Dinâmico Detectado na célula {current_cell} com {dynamic_conflict_id}")
                        return {
                            'success': False, 
                            'reason': 'DYNAMIC_CONFLICT', 
                            'obstacle_id': dynamic_conflict_id, 
                            'cell_id': current_cell,
                            'time_overlap': current_eta # Útil para debug
                        }
                    else:
                        print(f"Obstáculo Estático Detectado na célula {current_cell}")
                        return {
                            'success': False, 
                            'reason': 'STATIC_OBSTACLE', 
                            'obstacle_id': obstacle_id, 
                            'cell_id': current_cell
                        }
                        
        elif current_cell == target_cell:
            # Check for success condition
            print("Encontrou a célula de término: ", current_cell)
            break  # Exit the loop to proceed to final path generation

        # --- Step 2: Calculate the next candidate step ---
        # print("Iteração:", iteration_count, "Vetor:", line_vector)
        candidate_cell, next_point = find_next_step_on_vector(
            current_point,
            current_cell,
            line_vector,
            current_res
        )

        if candidate_cell is None:
            return {
                'success': False,
                'reason': 'Path is blocked, could not find a valid next step.',
                'obstacle_id': None,
                'cell_id': current_cell
            }

        # --- Step 3: Check the candidate cell ---
        # candidate_restriction_ratio = obstacle_model.get_ratio(candidate_cell, current_res)

        # Path is considered clear for this step
        # --- Step 4: Advance and attempt to Zoom-Out ---
        # Try to zoom out for faster travel if the parent cell is also clear
        try:
            parent_res = current_res - 1
            parent_cell = h3.cell_to_parent(candidate_cell, parent_res)
            parent_restriction_ratio = obstacle_model.get_ratio(parent_cell, parent_res, alt_level, current_eta)

            if current_res > MIN_RESOLUTION and parent_restriction_ratio == 0:
                # Zooom out logic moving to parent
                print("ZOOM-OUT Iteração:", iteration_count, "Resolução:", current_res, "Célula:", current_cell)
                current_res = parent_res
                current_point = next_point
                continue
        except:
            # In case of H3 error (e.g., pentagon), just proceed without changing resolution
            print("ERRO H3 Iteração:", iteration_count, "Resolução:", current_res, "Célula:", current_cell)
            pass

        # Finalize the move
        print("END DO DEF Iteração:", iteration_count, "Resolução:", current_res, "Célula:", current_cell)
        current_point = next_point

    # --- Step 5: Final path generation and return ---
    # This block is reached only if the while loop breaks on success
    try:
        return {'success': True}
    except Exception as e:
        return {
            'success': False,
            'reason': f'Final path generation failed after finding clear route: {e}',
            'obstacle_id': None,
            'cell_id': current_cell
        }