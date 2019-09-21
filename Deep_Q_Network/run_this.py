from grid_env import Grid
from RL_brain import DeepQNetwork
import numpy as np

N_AGENT = 10 # agent number
    
def run_grid():
    step = 0
    action = np.zeros(N_AGENT)
    
    for episode in range(300):
        # initial observation
        count = 0
        observation = env.reset()
        done = [False]*N_AGENT
        print('New episode: ', observation)
        while True:
            # fresh env
            env.render()

            for i in range(N_AGENT):
                # RL choose action based on observation
                if done[i]:
                    action[i] = 4
                else:
                    action[i] = RL[i].choose_action(observation[:, i])
                    for j in range(i):
                        if (abs(observation[0,i] - observation[0,j]) == 1 and \
                        observation[1,i] == observation[1,j]) or \
                            (observation[0,i] == observation[0,j] and \
                        abs(observation[1,i] - observation[1,j]) == 1):
                                action[i] = action[j].copy()
                

            # RL take action and get next observation and reward
            observation_, reward, done = env.step(action)

            for i in range(N_AGENT):
                RL[i].store_transition(observation[:, i], action[i], reward[i], observation_[:, i])

                if (step > 200) and (step % 5 == 0):
                    RL[i].learn()
                #if done[i]:
                    #action[i] = 5

            # swap observation
            observation = observation_.copy()

            # break while loop when end of this episode
            if done.all() or count > 1000:
                break
            step += 1
            count += 1
                

    # end of game
    print('game over')
    env.destroy()


if __name__ == "__main__":
    

    # maze game
    env = Grid(N_AGENT)
    RL = [DeepQNetwork(env.n_actions, env.n_features,
                      learning_rate=0.01,
                      reward_decay=0.9,
                      e_greedy=0.9,
                      replace_target_iter=200,
                      memory_size=2000,
                      # output_graph=True
                      )
                 ]*N_AGENT
    
    env.after(100, run_grid)
    env.mainloop()
    RL[0].plot_cost()
    