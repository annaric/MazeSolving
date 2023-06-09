from ThymioControl import ThymioControl
from math import fabs
import time
import numpy as np


class ThymioControlM(ThymioControl):
    ''' Thymio Control Class for the Mobile Robots Lab -- Moves the robot by setting the Pose in simulation'''
    def north(self,distance):
        self.sim.setObjectOrientation(self.handles[self.names['robot'][0]],self.sim.handle_world,[0,0,1.5708])
        prox=self.getProximity()
        if (prox > 0):
            return False
        else:
            p1=self.sim.getObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world)
            p1[1] = (round(p1[1],2)) + distance
            self.sim.setObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world,p1)
            return True
    
    def east(self,distance):  
        self.sim.setObjectOrientation(self.handles[self.names['robot'][0]],self.sim.handle_world,[0,0,0])
        prox=self.getProximity()
        if (prox > 0):
            return False
        else:
            p1=self.sim.getObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world)
            p1[0] = (round(p1[0],2)) + distance
            self.sim.setObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world,p1)
            return True
    
    def south(self,distance):       
        self.sim.setObjectOrientation(self.handles[self.names['robot'][0]],self.sim.handle_world,[0,0,-1.5708])
        prox=self.getProximity()
        if (prox > 0):
            return False
        else:
            p1=self.sim.getObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world)
            p1[1] = (round(p1[1],2)) - distance
            self.sim.setObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world,p1)
            return True
    
    def west(self,distance):
        self.sim.setObjectOrientation(self.handles[self.names['robot'][0]],self.sim.handle_world,[0,0,3.14159])
        prox=self.getProximity()
        if (prox > 0):
            return False
        else:
            p1=self.sim.getObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world)
            p1[0] = (round(p1[0],2)) - distance
            self.sim.setObjectPose(self.handles[self.names['robot'][0]],self.sim.handle_world,p1)
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
    def north(self,distance):
        lastMove = self.getMove()
        if lastMove == "EAST":
            self.turn(-90)
        elif lastMove == "SOUTH":
            self.turn(180)
        elif lastMove == "WEST":
            self.turn(90)
        
        prox=self.getProximity()
        self.setMove("NORTH")
        if (prox > 0):
            return False
        else:
            self.forward(distance)
            return True
        
    def east(self,distance):
        lastMove = self.getMove()
        if lastMove == "NORTH":
            self.turn(90)
        elif lastMove == "SOUTH":
            self.turn(-90)
        elif lastMove == "WEST":
            self.turn(180)

        prox=self.getProximity()
        self.setMove("EAST")
        if (prox > 0):
            return False
        else:
            self.forward(distance)
            return True

    def south(self,distance):
        lastMove = self.getMove()
        if lastMove == "NORTH":
            self.turn(180)
        elif lastMove == "EAST":
            self.turn(90)
        elif lastMove == "WEST":
            self.turn(-90)

        prox=self.getProximity()
        self.setMove("SOUTH")
        if (prox > 0):
            return False
        else:
            self.forward(distance)
            return True
    
    def west(self,distance):
        lastMove = self.getMove()
        if lastMove == "NORTH":
            self.turn(-90)
        elif lastMove == "EAST":
            self.turn(180)
        elif lastMove == "SOUTH":
            self.turn(90)

        prox=self.getProximity()
        self.setMove("WEST")
        if (prox > 0):
            return False
        else:
            self.forward(distance)
            return True

    def forward(self,distance):
        '''Robot moves forward a given distance'''
        speed=1.0 #deg/s
        dist_m = distance/100
        ang_speed = speed * math.pi/180 #rad/s
        lin_speed = ang_speed* 0.022 #wheel radius
        torque = 1 #N/m
        fwd_duration = dist_m/(lin_speed*torque)-1.6
        self.setSpeeds(speed,speed)
        start=self.sim.getSimulationTime()
        while self.sim.getSimulationTime()-start<fwd_duration:
            time.sleep(self.timestep/100)
        self.setSpeeds(0,0)


    def turn(self,angle):
        speed=1.0 #deg/s
        turn_duration=fabs(angle)/fabs(speed)*0.03775
        start=self.sim.getSimulationTime()      
        if angle>0:
            self.setSpeeds(-speed,speed)
        else:
            self.setSpeeds(speed,-speed)
        while self.sim.getSimulationTime()-start<turn_duration:
            time.sleep(self.timestep/100)
        self.setSpeeds(0,0)

import stable_baselines3.common.env_checker
from stable_baselines3 import DQN
from ThymioEnv import ThymioEnv


""" Commented area used for training

env=ThymioEnv(robot=ThymioControlC(),goal=10)
stable_baselines3.common.env_checker.check_env(env,warn=True)

env.robot.sim_speed=int(4096)
env.robot.display=True
env.robot.sim.setBoolParam(env.robot.sim.boolparam_display_enabled,env.robot.display)

# for the logs install tensorflow and execute tensorboard --logdir tb_logs in the ./tb_logs/ folder
model = DQN("MlpPolicy", env, verbose=1, tensorboard_log='./tb_logs/')
model.learn(total_timesteps=100000, log_interval=1)
model.save("move_robot_c")
del model

"""

model = DQN.load("move_robot_c")

print("Model trained succesful! Let's just try it out!")

env=ThymioEnv(robot=ThymioControlM(),goal=10) 
env.robot.sim_speed=int(1)
obs = env.reset()
env.robot.display=True
env.robot.sim.setBoolParam(env.robot.sim.boolparam_display_enabled,env.robot.display)

StepsArray = []
done = False

while not done:
    action, _states = model.predict(obs, deterministic=True)
    print(_states)
    
    obs, reward, done, info = env.step(action)    
    print(f'Observation: {obs} Action: {action}, Reward: {reward}, Done: {done}, Info: {info}')
    StepsArray.append(action)
    
    if done:
      obs = env.reset()
      done = True

a = np.array(StepsArray)
np.savetxt('StepArray.txt', a, fmt='%d')
b = np.loadtxt('StepArray.txt', dtype=int)
print(b)
