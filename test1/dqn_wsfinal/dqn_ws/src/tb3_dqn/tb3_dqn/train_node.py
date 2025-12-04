import rclpy
import sys
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
    
    EPISODES = 300
    
    print("SISTEMA DQN: MODO HAMBRE DE MOVIMIENTO")

    try:
        for e in range(EPISODES):
            print(f"Iniciando Episodio {e}")
            raw_scan = env.reset_env()
            dist, angle = env.get_goal_info()
            
            if raw_scan is None: continue
            state = processor.process_scan(raw_scan, dist, angle)
            total_reward = 0
            done = False
            steps = 0
            
            while not done:
                action = agent.act(state)
                next_scan, dist, angle, reward, done = env.step(action)
                
                if next_scan is None: continue
                next_state = processor.process_scan(next_scan, dist, angle)
                
                agent.remember(state, action, reward, next_state, done)
                agent.replay()
                
                state = next_state
                total_reward += reward
                steps += 1
                
                if steps > 500:
                    print("Tiempo agotado.")
                    done = True

            print(f"Episodio: {e} | Puntos: {total_reward:.2f} | Epsilon: {agent.epsilon:.2f}")
            if e % 10 == 0: agent.save()

    except KeyboardInterrupt:
        print("Detenido.")
    
    finally:
        agent.save()
        env.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()