import numpy as np
import random
from sklearn.neural_network import MLPRegressor
from collections import deque
import pickle
import os

class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        
        self.gamma = 0.99
        self.epsilon = 1.0
        self.epsilon_min = 0.05
        self.epsilon_decay = 0.995 
        
        self.learning_rate = 0.001
        self.batch_size = 64
        self.memory = deque(maxlen=10000)

        self.model = MLPRegressor(hidden_layer_sizes=(64, 64), 
                                  activation='relu', 
                                  solver='adam', 
                                  learning_rate_init=self.learning_rate,
                                  max_iter=1, 
                                  warm_start=True)
        dummy_input = np.zeros((1, state_size))
        dummy_target = np.zeros((1, action_size))
        self.model.fit(dummy_input, dummy_target)

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        q_values = self.model.predict(state.reshape(1, -1))
        return np.argmax(q_values[0])

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def replay(self):
        if len(self.memory) < self.batch_size:
            return

        minibatch = random.sample(self.memory, self.batch_size)
        states = np.array([i[0] for i in minibatch])
        next_states = np.array([i[3] for i in minibatch])
        
        targets = self.model.predict(states)
        next_q = self.model.predict(next_states)
        
        for i, (state, action, reward, next_state, done) in enumerate(minibatch):
            target = reward
            if not done:
                target = reward + self.gamma * np.amax(next_q[i])
            targets[i][action] = target
        
        self.model.partial_fit(states, targets)

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def save(self):
        print("Guardando modelo nuevo...")
        with open('trained_model.pkl', 'wb') as f:
            pickle.dump(self.model, f)