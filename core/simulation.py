import numpy as np
import pybullet as p
from typing import List, Dict, Any, Generator

# Bibliotecas do Gym-Pybullet-Drones
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from geopy.distance import geodesic
from core.models import FlightIntent

class CoordinateHelper:
    def __init__(self, origin_lat: float, origin_lon: float, origin_alt: float = 0):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.origin_alt = origin_alt

    def geo_to_cartesian(self, lat: float, lon: float, alt: float) -> np.ndarray:
        # Y (Norte)
        dist_y = geodesic((self.origin_lat, self.origin_lon), (lat, self.origin_lon)).meters
        y = dist_y if lat >= self.origin_lat else -dist_y
        # X (Leste)
        dist_x = geodesic((self.origin_lat, self.origin_lon), (self.origin_lat, lon)).meters
        x = dist_x if lon >= self.origin_lon else -dist_x
        # Z (Altitude)
        z = max(alt, 0.05)
        return np.array([x, y, z])

    def cartesian_to_geo(self, x: float, y: float, z: float) -> Dict[str, float]:
        METERS_PER_DEGREE_LAT = 111132.0
        d_lat = y / METERS_PER_DEGREE_LAT
        new_lat = self.origin_lat + d_lat
        radius_at_lat = METERS_PER_DEGREE_LAT * np.cos(np.radians(self.origin_lat))
        d_lon = x / radius_at_lat
        new_lon = self.origin_lon + d_lon
        return {"lat": new_lat, "lon": new_lon, "alt": z}

class DroneSimulationBackend:
    def __init__(self, intents: List[FlightIntent]):
        self.intents = intents
        if not intents:
            raise ValueError("A simulação requer pelo menos uma intenção de voo.")

        # 1. Setup de Coordenadas
        first_wp = intents[0].waypoints[0]
        self.coord_helper = CoordinateHelper(first_wp.lat, first_wp.lon, 0.0)

        self.start_pos = []
        self.start_orn = []
        self.trajectories = {}

        # Estruturas auxiliares para métricas 
        self.cruise_alts = []       # altitude de cruzeiro planejada de cada drone
        self.climb_end_idx = []     # índice onde termina a subida (por drone)
        self.descent_start_idx = [] # índice onde começa a descida (por drone)
        HOVER_STEPS = 100           # número de steps de hover inicial

        # 2. Planejamento de Trajetórias
        for i, intent in enumerate(intents):
            origin_wp = intent.waypoints[0]
            dest_wp   = intent.waypoints[-1]

            ground_alt = 0.05
            # Garante altitude segura de cruzeiro
            cruise_alt = max(origin_wp.alt, dest_wp.alt, 5.0) # altitude de cruzeiro planejada

            # Pontos Chave
            origin_ground = self.coord_helper.geo_to_cartesian(origin_wp.lat, origin_wp.lon, ground_alt)
            origin_cruise = self.coord_helper.geo_to_cartesian(origin_wp.lat, origin_wp.lon, cruise_alt)
            dest_cruise = self.coord_helper.geo_to_cartesian(dest_wp.lat, dest_wp.lon, cruise_alt)
            dest_ground = self.coord_helper.geo_to_cartesian(dest_wp.lat, dest_wp.lon, ground_alt)

            self.start_pos.append(origin_ground)
            self.start_orn.append(p.getQuaternionFromEuler([0, 0, 0]))

            # Rota: Decolagem -> Cruzeiro -> Pouso
            sparse_wps = [origin_ground, origin_cruise, dest_cruise, dest_ground]

            # Interpolação Fina (1.0m) para evitar inclinações agressivas
            dense_path = self._interpolate_path(sparse_wps, step_size=1.0)
            
            # Warmup (Hover inicial em solo na origem)
            start_hover = [origin_ground] * HOVER_STEPS
            path = start_hover + dense_path
            self.trajectories[i] = path

            # Guarda a altitude de cruzeiro para este drone
            self.cruise_alts.append(cruise_alt)

            # --- PRÉ-CÁLCULO DOS ÍNDICES DE FASE (por drone) ---
            alt_margin = 0.5  # tolerância em torno da altitude de cruzeiro
            climb_end = HOVER_STEPS  # default
            for k in range(HOVER_STEPS, len(path)):
                if path[k][2] >= cruise_alt - alt_margin:
                    climb_end = k
                    break

            descent_start = len(path) - 1
            for k in range(len(path) - 1, HOVER_STEPS, -1):
                if path[k][2] >= cruise_alt - alt_margin:
                    descent_start = k
                    break

            # sanidade: garante que descida começa depois do fim da subida
            if descent_start <= climb_end:
                descent_start = min(len(path) - 1, climb_end + 1)

            self.climb_end_idx.append(climb_end)
            self.descent_start_idx.append(descent_start)

        # Guarda o número de steps de hover (igual p/ todos)
        self.hover_steps = HOVER_STEPS

        self.start_pos = np.array(self.start_pos)
        self.start_orn = np.array(self.start_orn)
        self.num_drones = len(intents)

        # 3. Inicialização do PyBullet (240Hz Física e Controle)
        print("Initializing PyBullet Environment...")
        self.env = CtrlAviary(
            drone_model=DroneModel.CF2X,
            num_drones=self.num_drones,
            initial_xyzs=self.start_pos,
            initial_rpys=np.zeros((self.num_drones, 3)),
            physics=Physics.PYB,
            neighbourhood_radius=10,
            pyb_freq=240,
            ctrl_freq=240,
            gui=False,
            record=False
        )

        self.controllers = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(self.num_drones)]

    def _interpolate_path(self, points: List[np.ndarray], step_size: float) -> List[np.ndarray]:
        path = []
        for i in range(len(points) - 1):
            p1, p2 = points[i], points[i+1]
            dist = np.linalg.norm(p2 - p1)
            if dist == 0: continue
            direction = (p2 - p1) / dist
            steps = int(dist / step_size)
            if i == 0: path.append(p1)
            for k in range(1, steps + 1):
                path.append(p1 + direction * (k * step_size))
        path.append(points[-1])
        return path

    def run(self) -> Generator[List[Dict[str, Any]], None, None]:
        current_traj_indices = [0] * self.num_drones
        
        # Parâmetros Físicos
        m, g, kf = self.env.M, 9.8, self.env.KF
        hover_rpm = np.sqrt(m * g / (4 * kf))
        max_rpm = 22000
        # Idle seguro para manter estabilidade na descida
        idle_rpm = 3000 

        action = np.ones((self.num_drones, 4)) * hover_rpm
        ctrl_dt = 1.0 / float(self.env.CTRL_FREQ)

        # Para medir as velocidades médias
        prev_positions = [None] * self.num_drones
        speed_stats = [
            {"sum_climb": 0.0, "n_climb": 0,
             "sum_cruise": 0.0, "n_cruise": 0}
            for _ in range(self.num_drones)
        ]
        
        steps = 0
        max_steps = 200000

        print(f"Simulação iniciada (Hover: {hover_rpm:.0f}, Idle: {idle_rpm})...")

        try:
            while steps < max_steps:
                obs, reward, terminated, truncated, info = self.env.step(action)
                should_yield = (steps % 20 == 0)
                telemetry_frame = []

                for i in range(self.num_drones):
                    # --- Extração Explícita do Estado ---
                    # CtrlAviary (Kinematics) State Vector:
                    # 0-3: Pos, 3-7: Quat, 7-10: RPY, 10-13: Vel, 13-16: AngVel, 16-20: RPM
                    state_vec = obs[i]
                    cur_pos = state_vec[0:3]
                    cur_quat = state_vec[3:7]
                    cur_vel = state_vec[10:13]
                    cur_ang_vel = state_vec[13:16]

                    # Navegação
                    traj = self.trajectories[i]
                    idx = current_traj_indices[i]
                    
                    # Proteção de índice
                    if idx >= len(traj): idx = len(traj) - 1
                    target_wp = traj[idx]

                    dist_to_wp = np.linalg.norm(cur_pos - target_wp)

                    # --- MÉTRICAS DE VELOCIDADE (3D) POR DRONE ---
                    inst_speed = 0.0
                    if prev_positions[i] is not None:
                        delta = cur_pos - prev_positions[i]
                        inst_speed = float(np.linalg.norm(delta) / ctrl_dt)  # m/s (total 3D)

                        cruise_alt = self.cruise_alts[i]
                        climb_end = self.climb_end_idx[i]
                        descent_start = self.descent_start_idx[i]

                        # Classificação de fase baseada só no índice da trajetória
                        if idx <= self.hover_steps:
                            phase = "hover"
                        elif idx <= climb_end:
                            phase = "climb"
                        elif idx < descent_start:
                            phase = "cruise"
                        else:
                            phase = "descent"

                        stats = speed_stats[i]
                        if phase == "climb":
                            stats["sum_climb"] += inst_speed
                            stats["n_climb"] += 1
                        elif phase == "cruise":
                            stats["sum_cruise"] += inst_speed
                            stats["n_cruise"] += 1

                    # Atualiza a posição anterior deste drone
                    prev_positions[i] = cur_pos.copy()

                    # Debug ocasional
                    # if steps % 100 == 0:
                    #     print(f"[DEBUG] step: {steps}")
                    #     print(f"  cur_pos: {cur_pos}")
                    #     print(f"  target_wp: {target_wp}")
                    #     print(f"  dist_to_wp: {dist_to_wp}")
                    #     print(f"  rpm[0]: {action[i, 0]}")

                    # Avanço de Waypoint (Lookahead curto)
                    if dist_to_wp < 1.0 and idx < len(traj) - 1:
                        current_traj_indices[i] += 1
                        idx = current_traj_indices[i]
                        target_wp = traj[idx]

                    # --- Controle PID com Estado Explícito ---
                    # Passamos os vetores extraídos manualmente para evitar ambiguidade
                    rpm, _, _ = self.controllers[i].computeControl(
                        control_timestep=ctrl_dt,
                        cur_pos=cur_pos,
                        cur_quat=cur_quat,
                        cur_vel=cur_vel,
                        cur_ang_vel=cur_ang_vel,
                        target_pos=target_wp,
                        target_vel=np.zeros(3) # Velocidade zero para estabilidade máxima
                    )

                    # --- Detecção e Correção de Capotamento (Anti-Tumble) ---
                    # Converte Quaternion para Matriz de Rotação para checar o vetor Z (Up)
                    rot_matrix = p.getMatrixFromQuaternion(cur_quat)
                    # O terceiro elemento da terceira coluna (índice 8 na lista plana) representa Z-Z
                    z_axis_alignment = rot_matrix[8] 
                    
                    # Se z_axis < 0, o drone está de cabeça para baixo. Se < 0.1, está quase a 90 graus.
                    if z_axis_alignment < 0.1:
                        # Reset de Emergência: Traz de volta à posição atual, mas nivelado
                        p.resetBasePositionAndOrientation(
                            self.env.DRONE_IDS[i],
                            cur_pos,
                            p.getQuaternionFromEuler([0,0,0]),
                            physicsClientId=self.env.CLIENT
                        )
                        # Boost para recuperar altura se estiver caindo
                        rpm = np.ones(4) * hover_rpm * 1.1

                    # Clamp Final
                    action[i, :] = np.clip(rpm, idle_rpm, max_rpm)

                    # Telemetria
                    if should_yield:
                        geo_pos = self.coord_helper.cartesian_to_geo(cur_pos[0], cur_pos[1], cur_pos[2])
                        
                        # Debug Ocasional
                        if i == 0 and steps % 500 == 0:
                            print(f"[D0] Stp:{steps} | Alt:{cur_pos[2]:.1f}m -> Tgt:{target_wp[2]:.1f}m | Dist:{dist_to_wp:.1f}m")

                        status = "STOPPED" if (idx == len(traj) - 1 and dist_to_wp < 1.0) else "FLYING"
                        telemetry_frame.append({
                            "flight_id": self.intents[i].flight_id,
                            "lat": geo_pos["lat"], 
                            "lon": geo_pos["lon"], 
                            "alt": geo_pos["alt"],
                            "speed": inst_speed,  # Velocidade enviada ao frontend
                            "battery": 100.0, 
                            "status": status
                        })

                if should_yield:
                    yield telemetry_frame
                
                steps += 1

        except Exception as e:
            print(f"Erro na simulação: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Calcula e imprime as métricas de velocidade por drone
            for i in range(self.num_drones):
                stats = speed_stats[i]
                climb_avg = stats["sum_climb"] / stats["n_climb"] if stats["n_climb"] > 0 else 0.0
                cruise_avg = stats["sum_cruise"] / stats["n_cruise"] if stats["n_cruise"] > 0 else 0.0
                print(
                    f"[METRICS] Drone {i}: "
                    f"velocidade média de subida = {climb_avg:.2f} m/s, "
                    f"velocidade média de cruise = {cruise_avg:.2f} m/s"
                )

            self.env.close()
            print("Simulação Encerrada.")