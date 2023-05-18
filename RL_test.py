from ThymioControl import ThymioControl
from math import fabs
import time


class ThymioControlM(ThymioControl):
    ''' Thymio Control Class for the Mobile Robots Lab -- Moves the robot by setting the Pose in simulation'''
    def north(self,distance):
        self.sim.setObjectOrientation(self.handles[self.names['robot'][0]],self.sim.handle_world,[0,0,1.5708])
        prox=self.getProximity()
        if (prox > 0):
            #print("wall in front. Went north")
            #print("prox: ", prox)
            return False
        else:
            #print("Went north")
            #print("prox: ", prox)
            p1=self.sim.getObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world)
            #print("p1",p1)
            p1[1] = (round(p1[1],2)) + distance
            #p1[1]+=distance
            self.sim.setObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world,p1)
            #print("new position", p1)
            return True
    
    def east(self,distance):  
        self.sim.setObjectOrientation(self.handles[self.names['robot'][0]],self.sim.handle_world,[0,0,0])
        prox=self.getProximity()
        if (prox > 0):
            #print("wall in front. Went east")
            #print("prox: ", prox)
            return False
        else:
            #print("Went east")
            #print("prox: ", prox)
            p1=self.sim.getObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world)
            #print("p1",p1)
            p1[0] = (round(p1[0],2)) + distance
            #p1[0]+=distance
            self.sim.setObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world,p1)
            #print("new position", p1)
            return True
    
    def south(self,distance):       
        self.sim.setObjectOrientation(self.handles[self.names['robot'][0]],self.sim.handle_world,[0,0,-1.5708])
        prox=self.getProximity()
        if (prox > 0):
            #print("wall in front. Went south")
            #print("prox: ", prox)
            return False
        else:
            #print("Went south")
            #print("prox: ", prox)
            p1=self.sim.getObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world)
            #print("p1",p1)
            p1[1] = (round(p1[1],2)) - distance
            #p1[1]-=distance
            self.sim.setObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world,p1)
            #print("new position", p1)
            return True
    
    def west(self,distance):
        self.sim.setObjectOrientation(self.handles[self.names['robot'][0]],self.sim.handle_world,[0,0,3.14159])
        prox=self.getProximity()
        if (prox > 0):
            #print("wall in front. Went west")
            #print("prox: ", prox)
            return False
        else:
            #print("Went west")
            #print("prox: ", prox)
            p1=self.sim.getObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world)
            #print("p1",p1)
            p1[0] = (round(p1[0],2)) - distance
            #p1[0]-=distance
            self.sim.setObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world,p1)
            #print("new position", p1)
            return True

    def check_valid_state(self): 
        return not self.getContacts(body='/Thymio') and not self.out_of_bounds(body='/Thymio')
    
    def out_of_bounds(self,body='/Thymio'):
        ''' Checks if the robot is out of bounds'''
        p=self.getPose()
        #print(f'Pose: {p}')
        return p[0]<-1 or p[0]>1 or p[1]<-1 or p[1]>1
      

class ThymioControlC(ThymioControl):
    '''Thymio Control Class for the Mobile Robots Lab -- Moves the robot using the wheels'''
    def forward(self,distance):
        '''Robot moves forward a given distance'''
        speed=1.0
        fwd_duration=distance/speed*42.4
        self.setSpeeds(speed,speed)
        start=self.sim.getSimulationTime()
        while self.sim.getSimulationTime()-start<fwd_duration:
            time.sleep(self.timestep/100)
        self.setSpeeds(0,0)
        #self.step()


    def turn(self,angle):
        speed=1.0
        turn_duration=fabs(angle)/fabs(speed)*0.037
        start=self.sim.getSimulationTime()      
        if angle>0:
            self.setSpeeds(-speed,speed)
        else:
            self.setSpeeds(speed,-speed)
        while self.sim.getSimulationTime()-start<turn_duration:
            time.sleep(self.timestep/100)
        self.setSpeeds(0,0)


    def check_valid_state(self): 
        return not self.getContacts(body='/Thymio') and not self.out_of_bounds(body='/Thymio')
    
    def out_of_bounds(self,body='/Thymio'):
        ''' Checks if the robot is out of bounds'''
        p=self.getPose()
        #print(f'Pose: {p}')
        return p[0]<-1 or p[0]>1 or p[1]<-1 or p[1]>1

import gym
import stable_baselines3.common.env_checker
from stable_baselines3 import DQN
from ThymioEnv import ThymioEnv


env=ThymioEnv(robot=ThymioControlM(),goal=10)
stable_baselines3.common.env_checker.check_env(env,warn=True)

env.robot.sim_speed=int(4096)
env.robot.display=True
env.robot.sim.setBoolParam(env.robot.sim.boolparam_display_enabled,env.robot.display)



'''
env.robot.north(0.5)
prox = env.robot.getProximity()
print("prox north",prox)
time. sleep(5)

env.robot.west(0.5)
prox = env.robot.getProximity()
print("prox west",prox)
time. sleep(5)

env.robot.east(0.5)
prox = env.robot.getProximity()
print("prox east",prox)
time. sleep(5)

env.robot.south(0.5)
prox = env.robot.getProximity()
print("prox south",prox)
time. sleep(5)
#'''


#"""
model = DQN("MlpPolicy", env, verbose=1, exploration_final_eps=0.1, learning_rate=0.001, exploration_initial_eps=1, exploration_fraction=0.4)
model.learn(total_timesteps=100000, log_interval=1)
model.save("move_robot_c")
del model # remove to demonstrate saving and loading
#"""
model = DQN.load("move_robot_c")

print("Model trained succesful! Let's just try it out!")

env=ThymioEnv(robot=ThymioControlM(),goal=10) #
env.robot.sim_speed=int(1)
obs = env.reset()
env.robot.display=True
env.robot.sim.setBoolParam(env.robot.sim.boolparam_display_enabled,env.robot.display)

while True:
    action, _states = model.predict(obs, deterministic=True)
    print(_states)
    
    obs, reward, done, info = env.step(action)    
    print(f'Observation: {obs} Action: {action}, Reward: {reward}, Done: {done}, Info: {info}')
    #env.render()
    if done:
      obs = env.reset()
  #""" 