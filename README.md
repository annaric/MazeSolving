### Maze Solving

With this code you can train a virtual robot to solve a Maze with the help of Reinforcement Learning.

Instructions:
  1. Open a simulation instance and load Maze.ttt
  2. Run RL_test and either:
    - download existing model and run it with ``model = DQN.load("move_robot_c")`` or
    - learn a new policy with ``model.learn(total_timesteps=100000, log_interval=1)``
  
After successfully training a robot to solve a maze:
1. Open the Thymio Suit and turn on your Thymio robot and connect him to your computer
2. Open RealLifeImplementation.ipynb and run all the defined steps from the beginning to the end
3. The second last Step will let your real-life robot solve the maze defined in the virtual environment in real life.


The steps the robot has to take to find its way out of the maze, can be found in the StepArray.txt file.
0: move north
1: move east
2: move south
3: move west


This Repository can be found on https://github.com/annaric/MazeSolving