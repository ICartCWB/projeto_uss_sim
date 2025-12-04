import os
import h3
from typing import List, Dict, Set, Any, Optional
from collections import defaultdict
from datetime import datetime, timedelta

from core.models import FlightIntent

# Bibliotecas para Geo processamento
from fastkml import kml
from pygeoif.geometry import Polygon as PygeoifPolygon, MultiPolygon as PygeoifMultiPolygon
from shapely.geometry import Polygon as ShapelyPolygon
from geopy.distance import geodesic

# ==========================================
# 1. LEITURA E PARSE DO KML
# ==========================================

def load_kml_to_shapely(kml_path: str) -> list:
    """
    Loads a KML file, parses it, and robustly extracts all Polygon geometries
    by traversing its nested structure, handling Documents, Folders, Placemarks,
    and MultiGeometry objects.

    Args:
        kml_path: The full path to the .kml file.

    Returns:
        A list of Shapely Polygon objects found in the KML file.
    """

    # --- Helper Function 1: Extracts Shapely Polygons from a geometry object ---
    def _extract_shapely_from_geometry(geometry) -> list:
        """
        Processes a geometry object from fastkml. If it's a Polygon, it converts it.
        If it's a MultiGeometry, it iterates through its sub-geometries and calls itself recursively.
        """
        shapely_geoms = []
        # Case 1: The geometry is the Polygon we want.
        if isinstance(geometry, PygeoifPolygon):
            exterior_coords = [(coords[0], coords[1]) for coords in geometry.exterior.coords]
            interior_coords_list = []
            for interior_boundary in geometry.interiors:
                interior_coords = [(coords[0], coords[1]) for coords in interior_boundary.coords]
                interior_coords_list.append(interior_coords)

            shapely_geoms.append(ShapelyPolygon(exterior_coords, interior_coords_list))

        # Case 2: The geometry is a container for other geometries.
        elif isinstance(geometry, PygeoifMultiPolygon):
            for sub_geom in geometry.geoms:
                # Recurse to handle nested geometries
                shapely_geoms.extend(_extract_shapely_from_geometry(sub_geom))

        return shapely_geoms

    # --- Helper Function 2: Traverses the KML feature tree (Folders, Documents) ---
    def _traverse_features(features: list) -> list:
        """
        Recursively traverses a list of KML features (Documents, Folders, Placemarks)
        and calls the geometry extraction helper for each Placemark.
        """
        all_polygons = []
        for feature in features:
            # If it's a container, dive deeper
            if isinstance(feature, kml.Document) or isinstance(feature, kml.Folder):
                all_polygons.extend(_traverse_features(list(feature.features)))
            # If it's a placemark, try to extract geometry from it
            elif isinstance(feature, kml.Placemark) and hasattr(feature, 'geometry'):
                all_polygons.extend(_extract_shapely_from_geometry(feature.geometry))
        return all_polygons

    # --- Main function logic ---
    if not os.path.exists(kml_path):
        print(f"Error: Arquivo KML não encontrado: '{kml_path}'")
        return []

    with open(kml_path, 'rb') as kml_file:
        kml_content = kml_file.read()

    # Create KML object and parse the string
    k = kml.KML().from_string(kml_content)

    # Start the traversal from the top-level features
    top_level_features = list(k.features)
    final_polygons = _traverse_features(top_level_features)

    return final_polygons

# ==========================================
# 2. CLASSE H3 OBSTACLE MODEL
# ==========================================

class H3ObstacleModel:
    """
    A container class to process and store a multi-resolution obstacle model
    based on the H3 DGGS. It accepts a base (highest) and a minimum (lowest)
    resolution and processes all levels in between.
    """
    def __init__(self, shapely_polygons: List[ShapelyPolygon], base_res: int, min_res: int):
        """
        Initializes and processes the entire obstacle data model.

        Args:
            shapely_polygons: A list of Shapely Polygon objects.
            base_res: The highest H3 resolution for detailed representation.
            min_res: The lowest (coarsest) H3 resolution to process for parent ratios.
        """
        print("Initializing H3ObstacleModel...")

        # Estrutura: Dict[h3_cell, List[Dict]]
        # Onde cada registro na lista é: {'flight_id': str, 'level': int, 'start_time': datetime, 'end_time': datetime}
        self.dynamic_grid: Dict[str, List[Dict[str, Any]]] = defaultdict(list)

        if min_res >= base_res:
            raise ValueError("Minimum resolution (min_res) must be less than base resolution (base_res).")

        # --- Store basic configuration ---
        self.base_resolution: int = base_res
        self.min_resolution: int = min_res
        # Automatically generate the list of parent resolutions to process
        self.parent_resolutions: List[int] = list(range(self.base_resolution - 1, self.min_resolution - 1, -1))

        # --- Initialize data structures ---
        self.obstacle_geometries: Dict[int, Dict[str, Any]] = {}
        self.base_obstacle_cells: Dict[int, int] = {}
        # The parent_ratios structure are complex:
        # Dict[resolution, Dict[h3_parent_index, {'ratio': float, 'obstacle_ids': set}]]
        self.parent_ratios: Dict[int, Dict[str, Dict[str, Any]]] = {}

        # =============================================================================
        # Dict[resolution, Dict[h3_parent_index, {'occupied_count': int, 'total_children': int}]]
        self.dynamic_parent_ratios: Dict[int, Dict[str, Dict[str, Any]]] = defaultdict(lambda: defaultdict(lambda: {'occupied_count': 0, 'total_children': 0}))

        # --- GRID DINÂMICO (4D) ---
        self.dynamic_grid: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
        
        # --- REGISTRO REVERSO PARA DEREGISTER OTIMIZADO ---
        # Mapeia: flight_id -> Set[cell_h3] (inclui base e pais)
        self.flight_registry: Dict[str, Set[str]] = defaultdict(set)
        # =============================================================================

        # --- Run the processing methods ---
        self._process_polygons(shapely_polygons)
        self._calculate_parent_ratios()
        print("Model built successfully.")

    def _process_polygons(self, shapely_polygons: List[ShapelyPolygon]) -> None:
        """
        Processes each polygon to populate the vector and base grid models.
        This method is fully compatible with h3-py v4.x.
        """
        print(f"Processing {len(shapely_polygons)} obstacle polygons...")
        for i, poly in enumerate(shapely_polygons):
            obstacle_id = i + 1
            self.obstacle_geometries[obstacle_id] = {
                'polygon': poly,
                'vertices': list(poly.exterior.coords)
            }

            # --- FIX for H3 v4.x ---
            # 1. Get the GeoJSON-like dictionary from the shapely polygon.
            geo_interface = poly.__geo_interface__

            # 2. Use the official H3 function to convert the dictionary into an H3Shape object.
            h3shape = h3.geo_to_h3shape(geo_interface)

            # 3. Pass the resulting H3Shape object to the fill function.
            h3_cells_for_poly = h3.h3shape_to_cells(h3shape, self.base_resolution)
            # -------------------------

            for cell in h3_cells_for_poly:
                self.base_obstacle_cells[cell] = obstacle_id

    def _calculate_parent_ratios(self) -> None:
        """
        Correctly calculates the restriction ratio based on the count of
        obstructed child cells and collects the contributing obstacle IDs.
        """
        print(f"Calculating parent cell data for resolutions {self.parent_resolutions}...")

        # 1. This dict will count the ACTUAL number of obstructed child cells for each parent.
        parent_obstructed_child_counts = {res: defaultdict(int) for res in self.parent_resolutions}

        # 2. This dict will collect the unique IDs of obstacles for each parent (for the tooltip).
        parent_obstructed_ids = {res: defaultdict(set) for res in self.parent_resolutions}

        # For each base obstacle cell, update its parent's data
        for cell, obs_id in self.base_obstacle_cells.items():
            for res in self.parent_resolutions:
                parent = h3.cell_to_parent(cell, res)
                # Increment the child count for this parent
                parent_obstructed_child_counts[res][parent] += 1
                # Add the obstacle ID to the set for this parent
                parent_obstructed_ids[res][parent].add(obs_id)

        # Initialize the final dictionary
        self.parent_ratios = {res: {} for res in self.parent_resolutions}

        # Now, iterate through the parent cells to calculate the final ratio
        for res, counts_data in parent_obstructed_child_counts.items():
            for parent_cell, obstructed_count in counts_data.items():
                # Get the total number of children a parent cell can have at the base resolution
                total_children = len(h3.cell_to_children(parent_cell, self.base_resolution))

                if total_children > 0:
                    # The ratio is now correctly calculated based on cell counts
                    ratio = obstructed_count / total_children

                    # Get the corresponding set of obstacle IDs for the tooltip
                    id_set = parent_obstructed_ids[res][parent_cell]

                    # Store both the correct ratio and the set of obstacle IDs
                    self.parent_ratios[res][parent_cell] = {
                        'ratio': ratio,
                        'obstacle_ids': id_set
                    }

    def get_obstacle_id_from_cell(self, h3_cell: str) -> int | None:
        """
        Returns the ID of the obstacle that a given H3 cell belongs to.
        """
        return self.base_obstacle_cells.get(h3_cell)

    def get_ratio(self, 
                  cell_id: str, 
                  resolution: int, 
                  alt_level: Optional[int] = None, 
                  check_time: Optional[datetime] = None) -> float:
        """
        Gets the combined restriction ratio (Static + Dynamic 4D) for a given cell at a specific resolution.
        Handles base resolution and parent resolutions.
        Returns 1.0 for Static, 2.0 for Dynamic Conflict, when in a full conflict.
        Returns 0.0 if the cell is not restricted.

        Args:
            cell_id: H3 cell index.
            resolution: Resolution to check.
            alt_level: Altitude level (1-4) for dynamic check (required only at BASE_RESOLUTION).
            check_time: Time of arrival for dynamic check (required only at BASE_RESOLUTION).
        """
        # --- 1. Verificação Estática (Prioridade Máxima) ---

        # 1.1 Base Resolution: Retorno imediato se houver conflito estático (1.0)
        if resolution == self.base_resolution:
            if cell_id in self.base_obstacle_cells:
                return 1.0 

        # 1.2 Parent Resolutions: Retorno imediato se houver sinal de Zoom-in estático (> 0.0)
        elif resolution in self.parent_ratios:
            ratio_info = self.parent_ratios[resolution].get(cell_id, 0.0)
            static_ratio = ratio_info.get('ratio', 0.0) if isinstance(ratio_info, dict) else ratio_info
            
            if static_ratio > 0.0:
                return static_ratio
        
        # --- 2. Verificação Dinâmica (Combinada: Altura, Tempo e Propagação) ---

        if alt_level is not None and check_time is not None:
            
            # 2.1 Verificação Dinâmica da BASE (Certeza Absoluta)
            if resolution == self.base_resolution:
                conflicting_id = self.check_dynamic_occupation(cell_id, alt_level, check_time)
                if conflicting_id is not None:
                    return 2.0 # Conflito Dinâmico Confirmado (Força Parada)
            
            # 2.2 Verificação Dinâmica em CÉLULAS PAI (Sinal de Zoom-In)
            elif resolution in self.parent_resolutions:
                
                # Para evitar loops caros (check_dynamic_occupation é pesado),
                # consultamos o ratio pré-calculado que foi propagado (Dynamic Parent Ratios)
                # que indica que um voo está em algum lugar abaixo deste pai.
                
                # Note: Assumindo que a estrutura self.dynamic_parent_ratios foi adicionada corretamente
                # no seu __init__ e register_flight.
                
                parent_data = self.dynamic_parent_ratios[resolution].get(cell_id)
                if parent_data and parent_data['total_children'] > 0:
                    # Checagem Simples: Se há algum registro na lista do pai,
                    # e o tempo é compatível, retorna o ratio.
                    
                    # Como o register_flight preenche o dynamic_grid com o registro de tempo, 
                    # podemos fazer um check de tempo simplificado no próprio dynamic_grid:
                    
                    # Se houver qualquer entrada dinâmica que colida no tempo/altura neste PAI, 
                    # retornamos o ratio de ocupação para forçar o Zoom-In.
                    
                    # Se o check_dynamic_occupation retornar um ID, significa que o tempo/altura colidem.
                    conflicting_id = self.check_dynamic_occupation(cell_id, alt_level, check_time)
                    
                    if conflicting_id is not None:
                        # Se colide, retornamos o ratio de ocupação agregada (entre 0.0 e 1.0)
                        dynamic_ratio = parent_data['occupied_count'] / parent_data['total_children']
                        return dynamic_ratio # Sinal de Zoom-In Dinâmico
        
        # 3. Livre (Se não retornou em 1 ou 2)
        return 0.0

    def get_obstacle_by_id(self, obstacle_id: int) -> Dict[str, Any] | None:
        """
        Returns the geometry dictionary for a given obstacle ID.
        """
        return self.obstacle_geometries.get(obstacle_id)
    
    def __repr__(self) -> str:
        """
        Provides a string representation of the model's summary.
        """
        return (
            f"<H3ObstacleModel:\n"
            f"  Base Resolution: {self.base_resolution}\n"
            f"  Min. Parent Resolution: {self.min_resolution}\n"
            f"  Total Obstacle Polygons: {len(self.obstacle_geometries)}\n"
            f"  Total Base Obstacle Cells: {len(self.base_obstacle_cells)}\n"
            f">"
        )
    
    def _get_altitude_level(self, altitude_m: float) -> int:
        """
        Converte altitude em metros para níveis discretos de voo.
        Nível 1: ~30m | Nível 2: ~60m | Nível 3: ~90m | Nível 4: ~120m
        """
        if altitude_m > 45: return 2
        elif altitude_m > 75: return 3
        elif altitude_m > 105: return 4
        else: return 1

    def register_flight(self, intent: 'FlightIntent', contingency_time_sec: int = 30) -> None:
        """
        Registra o voo bloqueando TODAS as células da rota (Base + Pais) 
        pelo período total da operação (Start -> End + Contingência).
        Calcula quais células serão ocupadas e em quais horários.
        PROPAGA a ocupação para as células-mãe de forma otimizada.
        Args:
            intent: O objeto FlightIntent.
            contingency_time_sec: Tempo extra (segundos) adicionado ao final da janela 
                                  de ocupação para cobrir atrasos ou imprevistos.
        """
        flight_id = intent.flight_id
        
        # 1. Validação de Células
        if not intent.intent_cells:
            print(f"AVISO: Voo {flight_id} sem células definidas em 'intent_cells'. Ignorando.")
            return

        # 2. Definição de Altitude
        # Assume altitude do primeiro waypoint como referência para o nível
        alt_level = self._get_altitude_level(intent.origin.alt)

        # 3. Cálculo da Duração Total (ETA)
        # Distância Geodésica Total
        total_distance_m = geodesic(
            (intent.origin.lat, intent.origin.lon), 
            (intent.destination.lat, intent.destination.lon)
        ).meters
        
        cruise_time_sec = total_distance_m / intent.speed_m_s
        
        # Duração Total = Subida + Cruzeiro + Descida
        total_duration_sec = (
            intent.ascent_duration_sec + 
            cruise_time_sec + 
            intent.descent_duration_sec
        )
        
        # 4. Definição da Janela de Bloqueio (Única para todas as células)
        start_time = intent.start_time
        end_time = start_time + timedelta(seconds=total_duration_sec + contingency_time_sec)

        occupancy_data = {
            'flight_id': flight_id,
            'level': alt_level,
            'start_time': start_time,
            'end_time': end_time
        }
        
        print(f"Registrando {flight_id}: {len(intent.intent_cells)} células. "
              f"Bloqueio: {start_time.time()} até {end_time.time()} (nv {alt_level})")

        # 5. Registro no Grid e Propagação
        # Usamos um Set temporário para garantir que registramos cada célula (pai ou base) apenas uma vez por voo
        cells_to_block = set()

        # 5.1 Estrutura Temporária de Contagem Dinâmica (necessária para calcular total_children corretamente)
        cells_for_ratio_calc = defaultdict(set) # Set para evitar contar a mesma célula base 2x por acidente

        # a. Adiciona células da resolução base
        for cell in intent.intent_cells:
            cells_to_block.add(cell)
            cells_for_ratio_calc[self.base_resolution].add(cell)
            
            # b. Adiciona células pai (Propagação Upward)
            current_parent = cell
            for res in self.parent_resolutions:
                current_parent = h3.cell_to_parent(cell, res)
                cells_to_block.add(current_parent)
                cells_for_ratio_calc[res].add(cell) # Armazena a célula filha na lista de contagem do pai
        
        # 6. Atualização dos Ratios Dinâmicos Hierárquicos
        for res in self.parent_resolutions:
            current_parent_cells = set(h3.cell_to_parent(c, res) for c in cells_for_ratio_calc[self.base_resolution])
            
            for parent_cell in current_parent_cells:
                # Otimização: calcula o total_children e a ocupação apenas uma vez por pai
                
                # Conta quantas células-filhas ocupadas nesta resolução base pertencem a este pai
                occupied_children = len([
                    c for c in cells_for_ratio_calc[self.base_resolution] 
                    if h3.cell_to_parent(c, res) == parent_cell
                ])

                # O total de filhos é um valor constante (e caro para recalcular)
                total_children = len(h3.cell_to_children(parent_cell, self.base_resolution))

                # Atualiza o modelo de ratio dinâmico
                self.dynamic_parent_ratios[res][parent_cell]['occupied_count'] += occupied_children
                self.dynamic_parent_ratios[res][parent_cell]['total_children'] = total_children # (É redundante, mas seguro)

        # 7. Efetivação do Bloqueio
        for cell in cells_to_block:
            # Adiciona ao grid de conflitos
            self.dynamic_grid[cell].append(occupancy_data)
            # Adiciona ao registro reverso para remoção rápida
            self.flight_registry[flight_id].add(cell)

        print(f">> Voo {flight_id} registrado. {len(cells_to_block)} chaves de grid ocupadas (Base + Pais).")

    def deregister_flight(self, flight_id: str) -> None:
        """
        Remove um voo do grid dinâmico de forma otimizada (O(1) relativo ao grid).
        """
        if flight_id not in self.flight_registry:
            print(f"Aviso: Tentativa de remover voo inexistente ou já removido ({flight_id}).")
            return

        print(f"Removendo voo {flight_id}...")
        
        # Recupera todas as células onde este voo deixou rastro
        affected_cells = self.flight_registry[flight_id]
        
        for cell in affected_cells:
            if cell in self.dynamic_grid:
                # Filtra a lista removendo a entrada deste voo
                # (Recria a lista mantendo quem NÃO é o flight_id)
                self.dynamic_grid[cell] = [
                    occ for occ in self.dynamic_grid[cell] 
                    if occ['flight_id'] != flight_id
                ]
                
                # Limpeza de chaves vazias para economizar memória
                if not self.dynamic_grid[cell]:
                    del self.dynamic_grid[cell]
        
        # Limpa o registro reverso
        del self.flight_registry[flight_id]
        print(f">> Voo {flight_id} removido com sucesso.")

    def check_dynamic_occupation(self, cell_h3: str, alt_level: int, 
                                    check_time: datetime, time_buffer_sec: int = 0) -> Optional[str]:
        """
        Verifica se há conflito dinâmico em uma célula específica.
        Retorna o ID do voo conflitante ou None.
        
        Args:
            time_buffer_sec: Margem extra de segurança para a verificação da NOVA rota.
        """
        if cell_h3 not in self.dynamic_grid:
            return None
            
        # Define a janela de tempo que a NOVA rota deseja usar
        req_start = check_time - timedelta(seconds=time_buffer_sec)
        req_end = check_time + timedelta(seconds=time_buffer_sec)
        
        # Itera sobre as ocupações existentes nesta célula
        for occupation in self.dynamic_grid[cell_h3]:
            # 1. Verifica Altura
            if occupation['level'] != alt_level:
                continue # Níveis diferentes não colidem
            
            # 2. Verifica Tempo (Interseção de Intervalos)
            # Lógica: (StartA <= EndB) e (EndA >= StartB)
            occ_start = occupation['start_time']
            occ_end = occupation['end_time']
            
            if (req_start <= occ_end) and (req_end >= occ_start):
                return occupation['flight_id'] # Conflito detectado!
                
        return None

    def remove_expired_flights(self):
        """
        Limpeza (Garbage Collection): Remove registros de voos que já terminaram.
        Deve ser chamado periodicamente pelo USS.
        """
        now = datetime.now()
        for cell in list(self.dynamic_grid.keys()):
            # Mantém apenas voos cujo fim é no futuro
            self.dynamic_grid[cell] = [
                occ for occ in self.dynamic_grid[cell] 
                if occ['end_time'] > now
            ]
            # Se a lista ficar vazia, remove a chave para economizar memória
            if not self.dynamic_grid[cell]:
                del self.dynamic_grid[cell]
    
# ==========================================
# 3. FUNÇÃO FACTORY (Para uso no app.py)
# ==========================================

def build_obstacle_model_from_kml(kml_path: str, base_res: int = 12, min_res: int = 5) -> H3ObstacleModel:
    """
    Função auxiliar que orquestra a leitura do KML e a construção do modelo.
    """
    print(f"--- Iniciando Construção do Modelo a partir de: {kml_path} ---")
    try:
        polygons = load_kml_to_shapely(kml_path)
        if not polygons:
            print("AVISO: Nenhum polígono encontrado no KML.")
        
        model = H3ObstacleModel(polygons, base_res, min_res)
        return model
    except Exception as e:
        print(f"ERRO CRÍTICO ao carregar modelo de obstáculos: {e}")
        # Retorna um modelo vazio para não quebrar o app completamente, se desejar
        return H3ObstacleModel([], base_res, min_res)