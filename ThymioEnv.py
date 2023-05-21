from abc import ABC, abstractmethod
import gym
import math

STEP_REWARD = -0.04 
ALREADY_VISITED_REWARD= -0.25
#Distance reward function reward = reward + (1 - (dist/6)**0.4) => distRew: [0, 0.52)  [0, 0.07, 0.15, 0.24, 0.35, 0.52]
REACHED_GOAL_REWARD = 1
WALL_REWARD = -0.75 
ENDLESS_LOOP_PREVENTION_THRESHHOLD = -12.5

class ThymioEnv(gym.Env):
    ''' OpenAI Gym environment for the Thymio robot '''
    def __init__(self,robot,goal=10):
        self.action_space = gym.spaces.discrete.Discrete(4)
        self.observation_space = gym.spaces.discrete.Discrete(25)
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
        alreadyVisited=False

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
        reward = reward + distRew

        if (obs) in self.visited:
            reward = ALREADY_VISITED_REWARD
            alreadyVisited=True
        else:
            self.visited.add(obs)
    
        #Check if reached goal
        if dist<0.25:
            reward = REACHED_GOAL_REWARD
            reached=True
            print(f"Yuhuuuu!! Reward: {reward}")
        else:
            reached=False
        
        if not actionSuccess:
            reward = WALL_REWARD
        
        obs=self._getObs()
        self.totalReward+= reward

        if reached==True:
            print("total reward: ",self.totalReward)

        if self.totalReward <= ENDLESS_LOOP_PREVENTION_THRESHHOLD: 
            reached=True
    
        return obs, reward, reached, { "dist": dist, "alreadyVisited": alreadyVisited, "noWallInFront": actionSuccess }

    def _getObs(self):
        ''' Returns the current observation: a 2D pose '''
        obs=0
        position = self.robot.sim.getObjectPose(self.robot.handles[self.robot.names['robot'][0]],self.robot.sim.handle_world)
        positionHorizontal = int(((round(position[0],2)) + 0.75) * 2)
        positionVertical = int(((round(position[1],2)) + 0.75) * 2)
        obs = positionHorizontal + positionVertical * 5
        return int(obs)

    
    def distance(self,p1, p2):
        p1Row = p1%5
        p1Col = p1//5
        p2Row = p2%5
        p2Col = p2//5
        return abs(p1Row - p2Row) + abs(p1Col - p2Col)
   
    def reset(self):
        ''' Resets the simulation, returns initial observation and info'''   
        self.steps=0
        self.robot.reset()
        self.visited = set()
        self.visited.add(14)
        self.totalReward=0              
        return self._getObs()
