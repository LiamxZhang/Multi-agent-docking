"""
Reinforcement learning multi-agent docking in a grid world.

Red rectangle:          agent.
Black rectangles:       obstacle       [reward = ].
Yellow bin circle:      target    [reward = ].
All other states:       ground      [reward = ].

This script is the environment part of this example.
The RL is in RL_brain.py.

View more on my tutorial page: 
"""
import numpy as np
import math
import time
import sys
if sys.version_info.major == 2:
    import Tkinter as tk
else:
    import tkinter as tk

UNIT = 10  # pixels
GRID_H = 40  # grid height
GRID_W = 30  # grid width

T_MOTION = 1 # moving time
T_DOCK = 1 # docking time

class Grid(tk.Tk, object):
    def __init__(self, n_a = 1):
        super(Grid, self).__init__()
        self.action_space = ['u', 'd', 'l', 'r', 'w']
        self.n_actions = len(self.action_space)
        self.n_features = 2 # neural network parameter
        self.n_agents = n_a # agent number
        self.title('grid world')
        self.geometry('{0}x{1}'.format(GRID_W * UNIT, GRID_H * UNIT))
        self._build_grid()

    def _build_grid(self):
        self.canvas = tk.Canvas(self, bg='white',
                           height=GRID_H * UNIT,
                           width=GRID_W * UNIT)

        # create grids
        for c in range(0, GRID_W * UNIT, UNIT):
            x0, y0, x1, y1 = c, 0, c, GRID_H * UNIT
            self.canvas.create_line(x0, y0, x1, y1)
        for r in range(0, GRID_H * UNIT, UNIT):
            x0, y0, x1, y1 = 0, r, GRID_W * UNIT, r
            self.canvas.create_line(x0, y0, x1, y1)

        # create origin
        origin = np.array([UNIT/2, UNIT/2])

        ## OBSTACLES 
        # User-defined: height, width 
        obstacle_coordinate = np.array([[GRID_W/2, 1, 2, 3, 4],
                                    [GRID_H/2, 1, 2, 3, 4]])       
        n_obstacle = np.size(obstacle_coordinate, 1)
        obstacle_center = origin.repeat(n_obstacle).reshape(2,n_obstacle) \
                    + UNIT * obstacle_coordinate
        self.obstacles = []
        for i in range(n_obstacle):
            self.obstacles.append(self.canvas.create_rectangle(
                    obstacle_center[0][i] - UNIT/2, obstacle_center[1][i] - UNIT/2,
                    obstacle_center[0][i] + UNIT/2, obstacle_center[1][i] + UNIT/2,
                    fill='black'))
        
        ## TARGET
        self.target = []
        count = 0
        for i in range(math.ceil(np.sqrt(self.n_agents))):
            for j in range(math.ceil(np.sqrt(self.n_agents))):
                current_target = np.array([[GRID_W/2 + i], [GRID_H/2 + j]])

                if True in np.logical_and.reduce(current_target == obstacle_coordinate, axis = 0):
                    pass
                else:
                    count += 1
                    target_center = origin + UNIT * np.array([current_target[0][0],
                                                              current_target[1][0]])
                    self.target.append(self.canvas.create_oval(
                            target_center[0] - UNIT/2, target_center[1] - UNIT/2,
                            target_center[0] + UNIT/2, target_center[1] + UNIT/2,
                            fill='yellow'))
                    
                if count == self.n_agents: break
            if count == self.n_agents: break
        
        ## AGENTS
        np.random.seed(2)
        agent_H = np.random.randint(0, GRID_H, size = self.n_agents)
        agent_W = np.random.randint(0, GRID_W, size = self.n_agents)
        self.agent_center = origin.repeat(self.n_agents).reshape(2,self.n_agents) \
                        + UNIT * np.array([agent_W, agent_H])

        self.agent = []
        for i in range(self.n_agents):
            self.agent.append(self.canvas.create_rectangle(
                    self.agent_center[0][i] - UNIT/2, self.agent_center[1][i] - UNIT/2,
                    self.agent_center[0][i] + UNIT/2, self.agent_center[1][i] + UNIT/2,
                    fill='red'))
        
        # pack all
        self.canvas.pack()
        self.distance = np.ones(self.n_agents) * 100
        
    def reset(self):
        self.update()
        time.sleep(0.1)

        for i in range(self.n_agents):
            self.canvas.delete(self.agent[i])
        
        self.agent = []
        for i in range(self.n_agents):
            self.agent.append(self.canvas.create_rectangle(
                    self.agent_center[0][i] - UNIT/2, self.agent_center[1][i] - UNIT/2,
                    self.agent_center[0][i] + UNIT/2, self.agent_center[1][i] + UNIT/2,
                    fill='red'))
        
        observation = np.zeros((2, self.n_agents))
        for i in range(self.n_agents):
            # observation is the agents' coordinates 
#            observation[:,i] = (np.array(self.canvas.coords(self.agent[i])[:2]) \
#                       - np.array(self.canvas.coords(self.target)[:2]))/(GRID_H*UNIT)
            observation[:,i] = (np.array(self.canvas.coords(self.agent[i])[:2]))/(UNIT)
        return observation
    
    def render(self):
        # time.sleep(0.01)
        self.update()
        
    def step(self, action):
        s_ = np.zeros((2, self.n_agents))
        t_ = np.zeros((2, self.n_agents))
        reward = np.zeros(self.n_agents)
        done = np.zeros(self.n_agents)
        for i in range(self.n_agents):
            s = self.canvas.coords(self.agent[i])
            base_action = np.array([0, 0])
            if action[i] == 0:   # up
                if s[1] > UNIT:
                    base_action[1] -= UNIT
            elif action[i] == 1:   # down
                if s[1] < (GRID_H - 1) * UNIT:
                    base_action[1] += UNIT
            elif action[i] == 2:   # right
                if s[0] < (GRID_W - 1) * UNIT:
                    base_action[0] += UNIT
            elif action[i] == 3:   # left
                if s[0] > UNIT:
                    base_action[0] -= UNIT
            elif action[i] == 4:   # wait
                pass
            
            self.canvas.move(self.agent[i], base_action[0], base_action[1])  # move agent
            
            next_coords = self.canvas.coords(self.agent[i])  # next state
            agent_exc = self.agent.copy()
            agent_exc.pop(i)
            # average distance to the target
            s_[:, i] = (np.array(next_coords[:2]))/(UNIT)
            t_[:, i] = (np.array(self.canvas.coords(self.target[i])[:2]))/(UNIT)
            new_distance = ((s_[0, i]-t_[0, i])**2+(s_[1, i]-t_[1, i])**2)**0.5
            # reward function
            if self.coincide(next_coords, self.target):
                reward[i] = 100
                done[i] = True
            elif self.coincide(next_coords, self.obstacles):
                reward[i] = -100
                done[i] = True
            elif self.coincide(next_coords, agent_exc):
                reward[i] = -100
                done[i] = True
            elif new_distance < self.distance[i]:
                reward[i] = 1
            elif new_distance > self.distance[i]:
                reward[i] = -1
            else:
                reward[i] = 0
                done[i] = False
        
#            s_[:, i] = (np.array(next_coords[:2]) \
#              - np.array(self.canvas.coords(self.target)[:2]))/(GRID_H*UNIT)
            self.distance[i] = new_distance
            
        return s_, reward, done

    def coincide(self, agent, obstacles):
        c = False
        n = len(obstacles)
        for i in range(n):
            if agent == self.canvas.coords(obstacles[i]):
                c = True
            else:
                pass
        return c
    
        
        
