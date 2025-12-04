import h3
import numpy as np
from geopy.distance import geodesic
from typing import List, Tuple, Optional

# =============================================================================
# 1. ALGORITMO DE PASSO VETORIAL (Robustez Geométrica)
# =============================================================================

def find_next_step_on_vector(
    current_point: np.ndarray,
    current_cell: str,
    line_vector: List[np.ndarray],
    resolution: int,
    tolerance_factor: float = 1.0001,
    grid_ring = 1
) -> Tuple[Optional[str], Optional[np.ndarray]]:
    """
    Calcula o próximo passo ao longo de uma linha vetorial, encontrando a célula 
    H3 vizinha mais relevante e o ponto correspondente projetado na linha.
    
    Utiliza projeção geométrica plana (lat/lon como x/y) para eficiência em 
    curtas distâncias (vizinhança imediata).

    Se não houver entrada em tolerance_factor será considerado uma reta de células.

    Args:
        current_point (np.array): Coordenadas atuais (lat, lon) na linha.
        current_cell (str): Índice H3 da célula onde o ponto atual reside.
        line_vector (list): Lista de dois pontos definindo o vetor [start_pos, end_pos].
        resolution (int): Resolução H3 atual para busca de vizinhos.
        tolerance_factor (float): Fator de tolerância para a distância perpendicular
                                  máxima permitida (define a "largura" do corredor de busca).
        grid_ring (int): Quantidade de anéis de células examinados a partir da célula índice.

    Returns:
        tuple: (next_cell_h3, next_point_coords) 
               Retorna (None, None) se não houver próximo passo válido ou se o fim for atingido.
    """
    # 1. Geometric Setup
    line_start, line_end = line_vector
    # Line direction vector
    v_line = line_end - line_start
    # Normalize the main vector for dot product
    v_line_norm = v_line / np.linalg.norm(v_line)
    # Magnitude squared, for normalization (avoids square root)
    v_line_mag_sq = np.dot(v_line, v_line)

    # If the line has zero length, there is nowhere to go
    if v_line_mag_sq == 0:
        return None, None
    
    # =========================================================================
    # 1.5 CORREÇÃO DE DRIFT (Recuo Geométrico)
    # =========================================================================
    # Verifica onde o current_point está projetado na linha.
    # Isso corrige casos onde o ZOOM deslocou o ponto ou erro numérico o tirou da rota.
    
    vec_start_to_current = current_point - line_start
    projection_t_current = np.dot(vec_start_to_current, v_line_norm)
    
    # Caso 1: O ponto recuou para trás do início da linha (comum após Zoom-Out/In drástico)
    # ou erro de precisão o deixou negativo.
    if projection_t_current < -1e-9: # Tolerância epsilon pequena
        # RESET DE SEGURANÇA: Traz de volta para o início da linha
        current_point = line_start
    
    # Caso 2: O ponto flutuou para fora da linha (erro lateral).
    # Nós o "puxamos" de volta para a linha vetorial exata para garantir 
    # que as próximas projeções sejam precisas.
    else:
        # Recalcula a posição projetada exata na linha baseada no t atual
        # Isso remove o "drift" lateral acumulado
        current_point = line_start + (projection_t_current * v_line_norm)

    # =========================================================================

    # 2. Get Candidates
    try:
        # h3.grid_ring returns a set.
        candidate_cells = h3.grid_ring(current_cell, grid_ring)
    except h3.H3RingError:
        # Occurs if the cell is on a pentagon edge at resolution 0
        return None, None

    # 3. Calculate Projections and Distances
    valid_candidates = []

    # =========================================================================
    # Obtém o centro da célula atual para comparação de Voronoi
    center_current = np.array(h3.cell_to_latlng(current_cell))
    # =========================================================================

    # Perpendicular distance filter: tolerance_factor x the average edge length at the current resolution.
    max_perp_dist_m = tolerance_factor * h3.average_hexagon_edge_length(resolution, unit='m')

    for cell in candidate_cells:
        # a. Get the cell center
        center_c = np.array(h3.cell_to_latlng(cell))

        # b. Directional Filter: P' must be "ahead" of current_point, using dot product
        # Create a vector from the current point to the candidate's center.
        v_candidate = center_c - current_point
        # If the dot product is not positive, the cell is in the wrong direction.
        if np.dot(v_line_norm, v_candidate) <= 0:
            continue
        # Vector from the start of the line to the center of the cell

        # c. Calculate the projected point, P'
        w_vec = center_c - line_start
        # Dot product to find the projection scalar 't'
        t = np.dot(w_vec, v_line) / v_line_mag_sq
        # Projected point P'
        projected_point_p = line_start + t * v_line

        # =========================================================================
        # Validação de Voronoi (Evita Loop Infinito)
        # Verifica se o ponto projetado está geometricamente mais próximo da candidata
        # do que da célula atual. Se estiver mais perto da atual, o h3.latlng_to_cell
        # vai devolver a própria célula atual, travando o algoritmo.
        # ESTE PONTO CONSIDERA O ANDAMENTO COMO UMA LINHA
        
        if grid_ring == 1 and tolerance_factor < 1.001:
            dist_to_candidate = np.linalg.norm(projected_point_p - center_c)
            dist_to_current = np.linalg.norm(projected_point_p - center_current)

            # Se for mais perto (ou igual) da atual, não é um passo válido de saída.
            if dist_to_candidate >= dist_to_current:
                continue
        # =========================================================================

        # d. Calculate the vector distance between current point and P'
        # This determines the order of the steps.
        dist_from_start_to_p = np.linalg.norm(projected_point_p - line_start)

        # e. Calculate the perpendicular distance between C and P'
        # This correctly handles meridian convergence and avoids inaccurate conversions.
        perp_dist_m = geodesic(center_c, projected_point_p).meters

        # f. Applies the distance filter and stores
        if perp_dist_m < max_perp_dist_m:
            valid_candidates.append({
                "cell": cell,
                "point": projected_point_p,
                "dist_from_start": dist_from_start_to_p,
                "perp_dist_m": perp_dist_m
            })

    # 4. Choose the Winner
    if not valid_candidates:
        return None, None

    # Choose the winner based on the smallest forward progress.
    # With the robust dot product filter, this now correctly selects the
    # immediate next cell along the path, preventing jumps.
    winner = min(valid_candidates, key=lambda x: x["dist_from_start"])

    return winner["cell"], winner["point"]

# =============================================================================
# 2. GERADOR DE CAMINHO CONTÍNUO (Preciso)
# =============================================================================

def generate_fine_res_cells(line_vector, base_resolution, tolerance_factor=1.01):
    """
    Generates a continuous path of H3 cells at a fixed resolution
    that represents a straight vector line.

    Args:
        line_vector (list): A list of two points [(start_lat, start_lon), (end_lat, end_lon)].
        base_resolution (int): The target H3 resolution for the final path.
        tolerance_factor (float): The tolerance factor to pass to the stepping function.

    Returns:
        list: A list of H3 cell indices representing the path, or None on failure.
    """
    start_point, end_point = [np.array(p) for p in line_vector]

    # Initialize the path generation
    current_point = start_point
    start_cell = h3.latlng_to_cell(start_point[0], start_point[1], base_resolution)
    end_cell = h3.latlng_to_cell(end_point[0], end_point[1], base_resolution)

    # Use a list to maintain order
    fine_path_cells = [start_cell]
    current_cell = start_cell

    # Safety break to prevent infinite loops
    max_steps = 1000
    step_count = 0

    # Loop until the destination cell is reached
    while current_cell != end_cell:
        # Use our validated function to take one step at the fixed resolution
        next_cell, next_point = find_next_step_on_vector(
            current_point,
            current_cell,
            [start_point, end_point], # The main vector remains the reference
            base_resolution
        )

        if next_cell is None:
            print("Error: find_next_step_on_vector failed during fine path generation.")
            return None # Return None if a step fails

        # Add the new cell if it's not already the last one in the list
        if next_cell != current_cell:
            fine_path_cells.append(next_cell)

        # Update state for the next iteration
        current_cell = next_cell
        current_point = next_point

        # Safety break
        step_count += 1
        if step_count > max_steps:
            print("Error: Max steps reached during fine path generation.")
            return None

    return fine_path_cells