import rclpy
import pickle 
import numpy as np
import os
import time
from tb3_dqn.environment import TurtleBot3Env
from tb3_dqn.dqn_agent import DQNAgent
from tb3_dqn.state_processor import StateProcessor

def main(args=None):
    rclpy.init(args=args)
    
    env = TurtleBot3Env()
    processor = StateProcessor()
    
    state_size = 14
    action_size = env.action_size
    agent = DQNAgent(state_size, action_size)
    
    # --- RUTA ESTRATÉGICA ---
    mis_puntos = [
        (1.5, 0.0),    # 1. Recto a fondo
        (1.5, -1.0),   # 2. Derecha fondo
        (0.5, -0.5),   # 3. Derecha centro
        (0.0, 0.0),    # 4. Volviendo
        (0.5, 0.5)     # 5. Izquierda suave
    ]
    
    model_path = 'trained_model.pkl'
    if os.path.exists(model_path):
        print(f"📂 Cargando modelo desde: {model_path}")
        with open(model_path, 'rb') as f:
            agent.model = pickle.load(f)
    else:
        print("❌ ¡ERROR! No modelo.")
        return

    agent.epsilon = 0.0
    
    NUM_EPISODES = 20
    success_count = 0
    
    print("🏁 INICIANDO PRUEBAS CON ASISTENCIA DE MOVIMIENTO...")

    try:
        for e in range(NUM_EPISODES):
            target_x, target_y = mis_puntos[e % len(mis_puntos)]
            print(f"\n--- Prueba {e+1}/{NUM_EPISODES} -> Ir a ({target_x}, {target_y}) ---")
            
            raw_scan = env.reset_env()
            env.goal_x = target_x
            env.goal_y = target_y
            
            time.sleep(0.5)
            
            if raw_scan is None:
                for _ in range(10):
                    rclpy.spin_once(env, timeout_sec=0.2)
                    if env.scan_data is not None:
                        raw_scan = env.scan_data
                        break
            
            if raw_scan is None: continue

            dist, angle = env.get_goal_info()
            state = processor.process_scan(raw_scan, dist, angle)
            done = False
            steps = 0
            
            # --- VARIABLE ANTI-GIROS ---
            spin_counter = 0
            
            while not done:
                # 1. Preguntar al cerebro qué hacer
                action = agent.act(state)
                
                # --- 2. LA INTERVENCIÓN (EL FIX) ---
                # Si la acción es girar (3 o 4) o girar suave (1 o 2) sin avanzar mucho
                if action != 0: 
                    spin_counter += 1
                else:
                    spin_counter = 0 # Si va recto, reseteamos contador
                
                # Si lleva 3 pasos girando a lo loco, ¡LE OBLIGAMOS A IR RECTO!
                if spin_counter >= 3:
                    # print("⚠️ ¡Empujando robot!") # Descomenta si quieres ver cuándo actúa
                    action = 0 # FORZAR ADELANTE
                    spin_counter = 0
                # -----------------------------------

                next_scan, dist, angle, reward, done = env.step(action)
                
                if next_scan is None: 
                    rclpy.spin_once(env, timeout_sec=0.1)
                    continue
                
                state = processor.process_scan(next_scan, dist, angle)
                steps += 1
                
                # Criterio de éxito generoso
                if reward >= 200 or dist < 0.55:
                    success_count += 1
                    print("   🏆 ¡OBJETIVO ALCANZADO!")
                    done = True 
                
                if steps > 600:
                    print("   ⌛ Tiempo agotado.")
                    done = True

    except KeyboardInterrupt:
        print("Detenido.")
        
    finally:
        success_rate = (success_count / (e+1)) * 100
        print("\n" + "="*30)
        print(f"RESULTADOS FINALES: {success_count} / {e+1}")
        print(f"Tasa de Éxito: {success_rate:.1f}%")
        print("="*30)
        env.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()