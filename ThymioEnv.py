from abc import ABC, abstractmethod
import gym
import math

STEP_REWARD = 0.3
ALREADY_VISITED_REWARD= -0.5
#Distance reward function reward = reward + (1 - (dist/6)**0.4) => distRew: [0, 0.52)  [0, 0.07, 0.15, 0.24, 0.35, 0.52]
REACHED_GOAL_REWARD = 5
WALL_REWARD = -1 #-0.75
ENDLESS_LOOP_PREVENTION_THRESHHOLD = -25
ENDLESS_LOOP_PREVENTION_REWARD = 0

class ThymioEnv(gym.Env):
    ''' OpenAI Gym environment for the Thymio robot '''
    def __init__(self,robot,goal=10):
        self.action_space = gym.spaces.discrete.Discrete(4)
        #change observation space
        self.observation_space = gym.spaces.discrete.Discrete(25)
        #self.observation_space = gym.spaces.Box(low=-5, high=5, shape=(2,))
        self.robot=robot
        self.steps=0
        self.goal=10
        self.totalReward=0
        self.visited = set()
        self.visited.add(14)

    def step(self, action):
        ''' Performs a simulation step, returns observation, reward, terminated, truncated, info '''
        reward=0.0
        actionSuccess=False

        #Choose action
        if action==0:                      
            actionSuccess=self.robot.north(0.5)
            reward = STEP_REWARD

        elif action==1:
            actionSuccess=self.robot.east(0.5)
            reward = STEP_REWARD

        elif action==2:
            actionSuccess=self.robot.south(0.5) 
            reward = STEP_REWARD

        elif action==3:
            actionSuccess=self.robot.west(0.5) 
            reward = STEP_REWARD
        
        self.robot.step()
        self.steps+=1

        obs=self._getObs()
        dist=self.distance(obs,self.goal)
        distRew = (1 - (dist/6)**0.4)
        #print(distRew)
        reward = reward + distRew

        if (obs) in self.visited:
            reward = ALREADY_VISITED_REWARD
        else:
            self.visited.add(obs)
            #print("visited array", self.visited)

        #Check if reached goal
        if dist<0.25:
            reward = REACHED_GOAL_REWARD
            reached=True
            print(f"Yuhuuuu!! Reward: {reward}")
        else:
            reached=False
        
        if not actionSuccess:
            reward = WALL_REWARD
            reached=True
        
        obs=self._getObs()
        #print("reward: ", reward)
        self.totalReward+= reward

        if reached==True:
            print("total reward: ",self.totalReward)

        #if self.totalReward <= ENDLESS_LOOP_PREVENTION_THRESHHOLD: 
            #reward = ENDLESS_LOOP_PREVENTION_REWARD
            #reached=True
    
        return obs, reward, reached, { "dist": dist }

    def _getObs(self):
        ''' Returns the current observation: a 2D pose '''
        #obs=self.observation_space.sample()
        #pose=self.robot.getPose()
        #obs[0]=pose[0]
        #obs[1]=pose[1]
        obs=0
        position = self.robot.sim.getObjectPose(self.robot.handles[self.robot.names['robot'][0]],self.robot.sim.handle_world)
        positionHorizontal = int(((round(position[0],2)) + 0.75) * 2)
        positionVertical = int(((round(position[1],2)) + 0.75) * 2)
        #print("positionHorizontal ",  positionHorizontal)
        #print("positionVertical ",  positionVertical)
        obs = positionHorizontal + positionVertical * 5
        #print("obs: ", obs)
        return int(obs)

    
    def distance(self,p1, p2):
        p1Row = p1%5
        p1Col = p1//5
        p2Row = p2%5
        p2Col = p2//5
        return abs(p1Row - p2Row) + abs(p1Col - p2Col)
        #Disrecte distance
        #return sum(abs(val1-val2) for val1, val2 in zip(p1,p2)) * 2  #### Manhatten distance
        #return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
   
    def reset(self):
        ''' Resets the simulation, returns initial observation and info'''   
        self.steps=0
        self.robot.reset()
        self.visited = set()
        self.visited.add(14)
        self.totalReward=0              
        return self._getObs()
