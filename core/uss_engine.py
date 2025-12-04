import copy
from datetime import timedelta
from typing import List, Dict, Any
from core.models import FlightIntent

# Definição das Camadas (Layers) e Alturas "Seguras" (Midpoints) para sugestão
LAYER_MAP = {
    1: 30.0,   # Layer 1 (<= 45): Sugere 30m
    2: 60.0,   # Layer 2 (> 45, <= 75): Sugere 60m
    3: 90.0,   # Layer 3 (> 75, <= 105): Sugere 90m
    4: 120.0   # Layer 4 (> 105): Sugere 120m
}

class USSEngine:
    def __init__(self, navigator_instance):
        """
        Args:
            navigator_instance: Instância da classe HCP (antigo tactical_navigator)
                                que contém o método .check_conflict()
        """
        self.operational_intents_db: List[FlightIntent] = [] 
        self.navigator = navigator_instance

    def _get_layer_from_altitude(self, altitude_m: float) -> int:
        """Retorna a camada discreta baseada na altura contínua."""
        if altitude_m > 105: return 4
        elif altitude_m > 75: return 3
        elif altitude_m > 45: return 2
        else: return 1

    def process_flight_request(self, intent: FlightIntent) -> Dict[str, Any]:
        """
        Fluxo principal de aprovação ou desconflito.
        """
        # 1. Verifica conflito na intenção original
        has_conflict = self.navigator.check_conflict(intent, self.operational_intents_db)

        if not has_conflict:
            # Caminho Feliz: Aprova
            intent.status = "APPROVED"
            self.operational_intents_db.append(intent)
            return {
                "status": "APPROVED", 
                "msg": "Rota aprovada e registrada com sucesso.",
                "data": intent
            }
        
        else:
            # 2. Resolução Estratégica: Calcula alternativas viáveis
            alternatives = self._generate_alternatives(intent)
            return {
                "status": "CONFLICT", 
                "msg": "Conflito detectado com operações existentes.", 
                "alternatives": alternatives
            }

    def _generate_alternatives(self, original_intent: FlightIntent) -> List[Dict[str, Any]]:
        """
        Gera propostas de resolução variando Altura (Discreta) e Tempo (Contínuo).
        """
        suggestions = []
        
        # --- Estratégia A: Alteração de Nível de Voo (Vertical) ---
        current_alt = original_intent.waypoints[0].alt  # Assume altitude constante na perna inicial
        current_layer = self._get_layer_from_altitude(current_alt)
        
        # Tenta todas as outras camadas, priorizando as adjacentes
        target_layers = [l for l in LAYER_MAP.keys() if l != current_layer]
        
        for layer in target_layers:
            alt_m = LAYER_MAP[layer]
            
            # Cria candidato clonado com nova altura
            candidate = self._create_candidate_intent(original_intent, alt_mod=alt_m)
            
            try:
                if not self.navigator.check_conflict(candidate, self.operational_intents_db):
                    suggestions.append({
                        "type": "altitude",
                        "layer": layer,
                        "value_display": f"{alt_m}m (Layer {layer})",
                        "description": f"Alterar altitude para {alt_m}m",
                        "intent_candidate": candidate # Objeto pronto para ser aprovado
                    })
            except Exception:
                # Se houver erro de cálculo (ex: geometria inválida), ignora esta alternativa
                continue

        # --- Estratégia B: Alteração Temporal (Time Shift) ---
        # Tenta atrasar a partida em janelas discretas (em minutos)
        time_shifts = [2, 4, 6, 8, 10]
        
        for minutes in time_shifts:
            delta = timedelta(minutes=minutes)
            
            # Cria candidato clonado com deslocamento temporal
            candidate = self._create_candidate_intent(original_intent, time_mod=delta)
            
            if not self.navigator.check_conflict(candidate, self.operational_intents_db):
                suggestions.append({
                    "type": "time",
                    "value_display": f"+{minutes} min",
                    "description": f"Atrasar partida em {minutes} minutos",
                    "intent_candidate": candidate
                })
                # Se achou uma janela curta (ex: 2 min), não precisa testar 30 min? 
                # Depende da regra de negócio. Aqui vou parar na primeira válida temporal para não poluir.
                break 

        return suggestions

    def _create_candidate_intent(self, original: FlightIntent, alt_mod=None, time_mod=None) -> FlightIntent:
        """
        Helper para clonar e modificar uma intenção de voo.
        """
        candidate = copy.deepcopy(original)
        
        for wp in candidate.waypoints:
            if alt_mod is not None:
                wp.alt = alt_mod
            if time_mod is not None:
                wp.timestamp += time_mod
                
        return candidate