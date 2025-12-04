import os
import time
import streamlit as st
import pydeck as pdk
import folium
from streamlit_folium import st_folium
from datetime import datetime, timedelta

# Imports de Configuração
import config

# Imports do Core
from core.uss_engine import USSEngine
from core.obstacle_model import build_obstacle_model_from_kml
from core.models import FlightIntent, Waypoint
from core.simulation import DroneSimulationBackend
from core.geo_algorithms import generate_fine_res_cells
from core.hcp import check_route_feasibility

# --- 1. CONFIGURAÇÃO DA PÁGINA E ESTILO ---
st.set_page_config(page_title="USS Simulator 4D", layout="wide", page_icon="🚁")

# CSS Hack para maximizar a área de visualização dos mapas
st.markdown("""
    <style>
        /* Reduz o padding superior padrão do Streamlit para ganhar espaço */
        .block-container {
            padding-top: 1rem;
            padding-bottom: 1rem;
            padding-left: 2rem;
            padding-right: 2rem;
        }
        
        /* Força o PyDeck (Simulação) a ocupar mais altura */
        div[data-testid="stDeckGlJsonChart"] {
            height: 80vh !important;
            min-height: 600px;
        }
        
        /* Ajuste fino para o container do mapa */
        div[data-testid="stDeckGlJsonChart"] > div {
            height: 100% !important;
        }
        
        /* Remove margens extras de títulos */
        h1, h2, h3 {
            margin-bottom: 0.5rem;
        }
    </style>
""", unsafe_allow_html=True)

# ==========================================
# 2. GESTÃO DE ESTADO (SESSION STATE)
# ==========================================

def init_session_state():
    if 'authenticated' not in st.session_state:
        st.session_state.authenticated = False
    
    if 'route_points' not in st.session_state:
        st.session_state.route_points = [] # Lista de [lat, lon]
    
    # Armazena o resultado da última submissão para persistir botões de alternativa
    if 'last_submission_result' not in st.session_state:
        st.session_state.last_submission_result = None
    
    if 'last_submitted_intent' not in st.session_state:
        st.session_state.last_submitted_intent = None
    
    if 'uss' not in st.session_state:
        pass

init_session_state()

# ==========================================
# 3. CAMADA DE AUTENTICAÇÃO
# ==========================================

def check_password():
    if st.session_state.password_input == config.APP_PASSWORD:
        st.session_state.authenticated = True
        st.session_state.password_input = ""
    else:
        st.error("Senha incorreta.")

if not st.session_state.authenticated:
    st.markdown("## 🔐 Acesso Restrito USS")
    st.text_input("Insira a senha de acesso:", type="password", key="password_input", on_change=check_password)
    st.button("Entrar", on_click=check_password)
    st.stop()

# ==========================================
# 4. CARREGAMENTO DE RECURSOS (PÓS-AUTH)
# ==========================================

@st.cache_resource
def load_resources():
    if not os.path.exists('assets'):
        os.makedirs('assets')
        
    return build_obstacle_model_from_kml(
        config.KML_FILE_PATH, 
        config.H3_BASE_RESOLUTION, 
        config.H3_MIN_RESOLUTION
    )

obstacle_model = load_resources()

class NavigatorAdapter:
    def __init__(self, obstacle_model):
        self.model = obstacle_model
    def check_conflict(self, intent, db):
        result = check_route_feasibility(intent, self.model)
        has_conflict = not result['success']
        if has_conflict: 
            # Salva a razão exata no objeto intent para exibir na UI
            intent.last_conflict_reason = result.get('reason', 'Unknown')
        return has_conflict

if 'uss' not in st.session_state:
    nav_adapter = NavigatorAdapter(obstacle_model)
    st.session_state.uss = USSEngine(nav_adapter)

# ==========================================
# 5. AUXILIARES VISUAIS
# ==========================================

def get_layer_color(altitude):
    """Retorna uma cor distinta para cada faixa de altitude (Layer)."""
    if altitude <= 45:
        return "#0000FF" # Azul Puro (Baixa)
    elif altitude <= 75:
        return "#800080" # Roxo/Violeta (Média)
    elif altitude <= 105:
        return "#FF0080" # Magenta (Alta)
    else:
        return "#FF0000" # Vermelho (Muito Alta)

# ==========================================
# 6. LÓGICA DAS PÁGINAS
# ==========================================

def render_operator_page():
    st.header("🗺️ Planejamento de Missão (Operador)")
    
    col_map, col_controls = st.columns([3, 1])
    
    with col_controls:
        st.subheader("Parâmetros")
        op_id = st.text_input("ID Operador", "A_Pearce")
        altitude = st.slider("Altitude (m)", 30, 120, config.DEFAULT_ALTITUDE, step=30)
        
        st.markdown("---")
        st.markdown("**Status da Rota:**")
        
        points = st.session_state.route_points
        if len(points) == 0:
            st.info("📍 Clique no mapa para definir a ORIGEM.")
        elif len(points) == 1:
            st.info("🏁 Clique no mapa para definir o DESTINO.")
        elif len(points) == 2:
            st.success("✅ Rota definida!")
            st.write(f"**Origem:** {points[0][0]:.4f}, {points[0][1]:.4f}")
            st.write(f"**Destino:** {points[1][0]:.4f}, {points[1][1]:.4f}")
            
            if st.button("Limpar Pontos"):
                st.session_state.route_points = []
                st.session_state.last_submission_result = None # Limpa estados antigos
                st.rerun()
                
            if st.button("🚀 Solicitar Aprovação", type="primary"):
                process_flight_submission(op_id, points[0], points[1], altitude)
        
        # --- ÁREA DE RESULTADOS E ALTERNATIVAS ---
        # Renderizada fora do bloco do botão para persistir após cliques
        if st.session_state.last_submission_result:
            result = st.session_state.last_submission_result
            intent = st.session_state.last_submitted_intent
            
            st.markdown("---")
            if result["status"] == "APPROVED":
                st.success(f"✅ Voo Aprovado!\n\nTempo de Cálculo: {result.get('calc_time', 0):.4f}s")
            
            elif result["status"] == "CONFLICT":
                # Lógica de Exibição de Erro Baseada no Motivo (Reason)
                reason = getattr(intent, 'last_conflict_reason', 'Desconhecido')
                
                if "STATIC_OBSTACLE" in reason:
                    st.error("🛑 **Conflito Estático Detectado:** A rota cruza uma Zona de Exclusão Aérea (NFZ).")
                elif "DYNAMIC_CONFLICT" in reason:
                    st.error("⚠️ **Conflito Dinâmico Detectado:** Risco de colisão com outra operação ativa.")
                elif "max iterations" in reason or "Path is blocked" in reason:
                    st.error("🚫 **Erro de Execução (HCP):** O algoritmo não encontrou caminho geométrico viável.")
                else:
                    st.error(f"❌ **Conflito Detectado**\n{reason}")

                st.caption(f"Tempo de Análise: {result.get('calc_time', 0):.4f}s")

                # Renderiza Sugestões se houver
                if "alternatives" in result and result["alternatives"]:
                    st.warning("💡 **Alternativas de Resolução:**")
                    
                    for i, alt in enumerate(result["alternatives"]):
                        # Botão para cada alternativa
                        if st.button(f"✅ {alt['description']}", key=f"btn_alt_{i}"):
                            accept_alternative(alt['intent_candidate'])

    with col_map:
        center_lat, center_lon = -23.516281, -46.624057 # Centro do Mapa: Rodoviária do Tietê
        
        m = folium.Map(location=[center_lat, center_lon], zoom_start=14)

        # --- A. DESENHO DAS RESTRIÇÕES (NFZ) ---
        if obstacle_model and hasattr(obstacle_model, 'obstacle_geometries'):
            for obs_id, data in obstacle_model.obstacle_geometries.items():
                poly = data['polygon']
                # Shapely usa (lon, lat), Folium usa (lat, lon). Precisamos inverter.
                locations = [(p[1], p[0]) for p in poly.exterior.coords]
                
                folium.Polygon(
                    locations=locations,
                    color="#FF4B4B",      # Vermelho alerta
                    weight=2,
                    fill=True,
                    fill_opacity=0.2,     # Transparente para ver a rua
                    popup=f"NFZ-{obs_id}",
                    tooltip="Zona de Restrição (NFZ)"
                ).add_to(m)
        
        # --- B. DESENHO DAS ROTAS APROVADAS (OPERACIONAL) ---
        if 'uss' in st.session_state:
            for flight in st.session_state.uss.operational_intents_db:
                path_coords = [(wp.lat, wp.lon) for wp in flight.waypoints]
                flight_alt = flight.waypoints[0].alt
                color = get_layer_color(flight_alt)
                
                folium.PolyLine(
                    locations=path_coords,
                    color=color,
                    weight=3,
                    opacity=0.7,
                    tooltip=f"Voo: {flight.flight_id} | Alt: {flight_alt}m",
                    dash_array='5, 5' # Tracejado para diferenciar da rota ativa
                ).add_to(m)

        # --- C. DESENHO DA ROTA EM PLANEJAMENTO ---
        for i, pt in enumerate(st.session_state.route_points):
            color = "green" if i == 0 else "red"
            label = "Origem" if i == 0 else "Destino"
            icon = folium.Icon(color=color, icon="plane", prefix="fa")
            folium.Marker(location=pt, popup=label, icon=icon).add_to(m)
            
        if len(st.session_state.route_points) == 2:
            folium.PolyLine(st.session_state.route_points, color="blue", weight=2.5, opacity=1).add_to(m)

        # Altura aumentada para preencher a tela
        output = st_folium(m, height=900, use_container_width=True)

        if output['last_clicked'] and len(st.session_state.route_points) < 2:
            new_point = [output['last_clicked']['lat'], output['last_clicked']['lng']]
            if not st.session_state.route_points or st.session_state.route_points[-1] != new_point:
                st.session_state.route_points.append(new_point)
                st.rerun()

def process_flight_submission(op_id, origin, dest, alt):
    """Processa a solicitação e salva o resultado no Session State."""
    
    start_time = datetime.now() + timedelta(seconds=10)
    
    wp1 = Waypoint(origin[0], origin[1], alt, start_time)
    wp2 = Waypoint(dest[0], dest[1], alt, start_time + timedelta(minutes=5))
    
    intent = FlightIntent(
        operator_id=op_id,
        flight_id=f"Flight_{int(time.time())}",
        waypoints=[wp1, wp2],
        start_time=start_time,
        speed_m_s=config.DEFAULT_SPEED 
    )

    # --- INÍCIO DO CRONÔMETRO (Cálculo Geométrico + Lógico) ---
    calc_start_time = time.time()
    
    line_vector = [tuple(origin), tuple(dest)]
    try:
        # 1. Cálculo Geométrico (Pesado)
        path_cells = generate_fine_res_cells(line_vector, config.H3_BASE_RESOLUTION)
        if not path_cells:
            st.error("Erro: Rota muito curta ou inválida para H3.")
            return
        intent.intent_cells = path_cells
    except Exception as e:
        st.error(f"Erro na geometria H3: {e}")
        return

    # 2. Verificação Lógica (HCP - Hierarchical Conflict Probe)
    response = st.session_state.uss.process_flight_request(intent)

     # --- FIM DO CRONÔMETRO ---
    calc_end_time = time.time()
    response['calc_time'] = calc_end_time - calc_start_time
    
    # Salva no estado para renderização persistente
    st.session_state.last_submission_result = response
    st.session_state.last_submitted_intent = intent

    if response["status"] == "APPROVED":
        obstacle_model.register_flight(intent)
        st.session_state.route_points = []
        st.balloons()
        # Não damos rerun imediato aqui para deixar o usuário ler o sucesso antes de limpar
    else:
        # Se for conflito, apenas atualiza a UI para mostrar as opções
        pass

def accept_alternative(candidate_intent):
    """Aplica uma alternativa selecionada."""
    # Aprova manualmente a candidata
    candidate_intent.status = "APPROVED"
    
    # Registra no banco de dados e no modelo físico
    st.session_state.uss.operational_intents_db.append(candidate_intent)
    obstacle_model.register_flight(candidate_intent)
    
    # Limpa estados de UI
    st.session_state.last_submission_result = None
    st.session_state.last_submitted_intent = None
    st.session_state.route_points = []
    
    st.success(f"Alternativa aceita! Voo {candidate_intent.flight_id} registrado.")
    time.sleep(1.0)
    st.rerun()

def render_simulation_page():
    st.header("🎮 Simulação em Tempo Real (PyDeck)")
    
    rotas = st.session_state.uss.operational_intents_db
    
    if not rotas:
        st.warning("Nenhuma rota aprovada para simular. Vá para a aba de Operador.")
        return

    # Layout de Controles Melhorado
    col_info, col_speed, col_action = st.columns([3, 2, 1])

    with col_info:
        st.write(f"**Rotas Ativas:** {len(rotas)}")
        st.write("O backend de física (PyBullet) irá calcular a trajetória real considerando massa, arrasto e PID.")
    
    with col_speed:
        # Slider de Time Multiplier
        # Permite acelerar a renderização visual da simulação
        time_multiplier = st.select_slider(
            "⏩ Velocidade do Tempo",
            options=[1, 2, 5, 10, 20, 50, 100],
            value=1,
            format_func=lambda x: f"{x}x"
        )

    with col_action:
        run_sim = st.button("▶️ INICIAR SIMULAÇÃO", type="primary")

    map_placeholder = st.empty()
    # status_text = st.empty()

    initial_lat = rotas[-1].waypoints[0].lat
    initial_lon = rotas[-1].waypoints[0].lon
    
    view_state = pdk.ViewState(
        latitude=initial_lat,
        longitude=initial_lon,
        zoom=15,
        pitch=50,
    )

    # Constrói os layers de rotas para o PyDeck também, com as mesmas cores
    path_data = []
    for flight in rotas:
        path = [[wp.lon, wp.lat, wp.alt] for wp in flight.waypoints]
        # Converte cor hex para RGB lista [R, G, B]
        hex_color = get_layer_color(flight.waypoints[0].alt).lstrip('#')
        rgb_color = list(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
        
        path_data.append({
            "path": path, 
            "id": flight.flight_id,
            "color": rgb_color
        })
        
    route_layer = pdk.Layer(
        "PathLayer",
        path_data,
        get_path="path",
        get_color="color", # Usa a cor dinâmica
        width_min_pixels=3,
        opacity=0.6
    )

    deck = pdk.Deck(
        layers=[route_layer],
        initial_view_state=view_state,
        map_style="mapbox://styles/mapbox/dark-v10" if config.MAPBOX_API_KEY else None
    )
    
    # Renderiza o deck. O CSS injetado no início forçará a altura deste elemento.
    map_placeholder.pydeck_chart(deck)

    if run_sim:
        sim_backend = DroneSimulationBackend(rotas)
        
        for frame in sim_backend.run():
            drone_data = []
            for d in frame:
                drone_data.append({
                    "pos": [d['lon'], d['lat'], d['alt']],
                    "id": d['flight_id'],
                    # Formatando dados para o tooltip
                    "alt": f"{d['alt']:.1f}m",
                    "speed": f"{d.get('speed', 0.0):.1f} m/s", # Pega a velocidade calculada
                    "status": d['status']
                })
            
            drone_layer = pdk.Layer(
                "ScatterplotLayer",
                drone_data,
                get_position="pos",
                get_fill_color=[255, 200, 0],
                get_radius=15,
                pickable=True,
                radius_min_pixels=5,
                radius_max_pixels=20
            )
            
            new_deck = pdk.Deck(
                layers=[route_layer, drone_layer],
                initial_view_state=view_state,
                map_style="mapbox://styles/mapbox/dark-v10" if config.MAPBOX_API_KEY else None,
                # CONFIGURAÇÃO DE TOOLTIP NO PYDECK
                tooltip={
                    "html": "<b>ID:</b> {id}<br/>"
                            "<b>Alt:</b> {alt}<br/>"
                            "<b>Vel:</b> {speed}<br/>"
                            "<b>Status:</b> {status}",
                    "style": {
                        "backgroundColor": "#1E1E1E",
                        "color": "#FFFFFF",
                        "fontSize": "14px",
                        "borderRadius": "5px",
                        "padding": "10px"
                    }
                }
            )
            
            map_placeholder.pydeck_chart(new_deck)
            
            # Aplica o multiplicador de tempo no delay
            # Se for 10x, o sleep será 0.005s em vez de 0.05s
            delay = config.SIMULATION_FPS / time_multiplier
            time.sleep(delay)

# ==========================================
# 7. MENU DE NAVEGAÇÃO
# ==========================================

st.sidebar.title("USS Menu")
page = st.sidebar.radio("Navegação", ["Operador (Input)", "Simulação (Live)"])

st.sidebar.markdown("---")
st.sidebar.info(f"Usuário Autenticado.\nAmbiente: {'Mapbox Ativo' if config.MAPBOX_API_KEY else 'Mapbox Inativo'}")

if page == "Operador (Input)":
    render_operator_page()
else:
    render_simulation_page()