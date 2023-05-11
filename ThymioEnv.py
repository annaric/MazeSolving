from abc import ABC, abstractmethod
import gym
import math

class ThymioEnv(gym.Env):
    ''' OpenAI Gym environment for the Thymio robot '''
    def __init__(self,robot,goal=[-0.8,0.2]):
        self.action_space = gym.spaces.discrete.Discrete(4)
        self.observation_space = gym.spaces.Box(low=-5, high=5, shape=(2,))
        self.robot=robot
        self.steps=0
        self.goal=[-0.8,0.2]

    def step(self, action):
        ''' Performs a simulation step, returns observation, reward, terminated, truncated, info '''

        reward=0.0
        actionSuccess=False
        #Choose action
        if action==0:                      
            actionSuccess=self.robot.north(0.5)
            #reward forward movements
            reward+=1.0

        elif action==1:
            actionSuccess=self.robot.east(0.5)
            reward+=1.0

        elif action==2:
            actionSuccess=self.robot.south(0.5) 
            reward+=1.0

        elif action==3:
            actionSuccess=self.robot.west(0.5) 
            reward+=1.0
        
        self.robot.step()
        self.steps+=1
        
        #Check distance to goal
        dist=self.distance(self.robot.getPose(),self.goal)
        #penalize distance to goal
        reward-=dist**2

        #Check if reached goal
        if dist<0.25:
            #reward for reaching the goal
            reward+=100.0
            reached=True
            print(f"Yuhuuuu!! Reward: {reward}")
        else:
            reached=False

        #Check proximity sensors
        prox=0
        #for p in self.robot.getProximity():
        #    prox+=p

        prox = self.robot.getProximity()
        #penalize proximity to obstacles
        reward-=prox           

        #Check if robot is in a valid state
        if not self.robot.check_valid_state():
            #the task failed
            reward-=1.0
            reached=True
        
        if not actionSuccess:
            #the task failed
            print("Task failed, no actionSuccess (wall)")
            reward-=1.0
            reached=True
        
        obs=self._getObs()
    
        return obs, reward , reached, {}

    def _getObs(self):
        ''' Returns the current observation: a 2D pose '''
        obs=self.observation_space.sample()
        pose=self.robot.getPose()
        obs[0]=pose[0]
        obs[1]=pose[1]
        return obs

    
    def distance(self,p1, p2):
        # return sum(abs(val1-val2) for val1, val2 in zip(p1,p2)) * 2  #### Manhatten distance later
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
   
    def reset(self):
        ''' Resets the simulation, returns initial observation and info'''   
        self.steps=0
        self.robot.reset()                
        return self._getObs()
