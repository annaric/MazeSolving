from abc import ABC, abstractmethod
import gym
import math

class ThymioEnv(gym.Env):
    ''' OpenAI Gym environment for the Thymio robot '''
    def __init__(self,robot,goal=[1,0.6]):
        #self.action_space = gym.spaces.Discrete(3)
        self.action_space = gym.spaces.discrete.Discrete(3)
        self.observation_space = gym.spaces.Box(low=-5, high=5, shape=(3,))
        self.robot=robot
        self.steps=0
        self.goal=[1,0.6]

    def step(self, action):
        ''' Performs a simulation step, returns observation, reward, terminated, truncated, info '''

        reward=0.0
        #Choose action
        if action==0:                      
            self.robot.forward(0.1)
            #reward forward movements
            reward+=1.0

        elif action==1:
            self.robot.turn(90)
            reward+=0.1

        elif action==2:
            self.robot.turn(-90) 
            reward+=0.1
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
        for p in self.robot.getProximity():
            prox+=p
        #penalize proximity to obstacles
        reward-=prox*1.0            

        #Check if robot is in a valid state
        if not self.robot.check_valid_state():
            #the task failed
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
        obs[2]=pose[2]
        return obs

    
    def distance(self,p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
   
    def reset(self):
        ''' Resets the simulation, returns initial observation and info'''   
        self.steps=0
        self.robot.reset()                
        return self._getObs()
