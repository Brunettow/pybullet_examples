#import pybullet as p
#import time
#import pybullet_data
#physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
#p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
#p.setGravity(0,0,-10)
#planeId = p.loadURDF("plane.urdf")
#cubeStartPos = [0,0,1]
#cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
#for i in range (10000):
#    p.stepSimulation()
#    time.sleep(1./240.)
#cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
#print(cubePos,cubeOrn)
#p.disconnect()


import os
import pybullet as p
import pybullet_data
import math
import time
import random
import numpy as np
print(random.uniform(0,1))

#import pybullet as p
#import time
#import numpy as np
#
## Initialize PyBullet simulation
#p.connect(p.GUI)
#p.setGravity(0, 0, -9.81)
#
## Load plane
#planeId = p.loadURDF("plane.urdf")
#
## Load cube
#cube_urdf = "cube.urdf"  # Replace with the path to your cube URDF
#cube_size = 0.2
#cube_mass = 1.0
#
## Create 10 cubes
#cubes = []
#for _ in range(10):
#    cube_pos = np.random.uniform(-1, 1, size=3)
#    cube_id = p.loadURDF(cube_urdf, cube_pos, globalScaling=cube_size)
#    p.changeDynamics(cube_id, -1, mass=cube_mass)
#    cubes.append(cube_id)
#
## Simulation loop
#for _ in range(1000):  # Run for 1000 timesteps
#    # Teleport a random cube above another random cube
#    
#    #select a random cube
#    selected_cube_index = np.random.randint(10)
#    
#    #select a cube that not selected above line
#    target_cube_index = np.random.choice(list(range(selected_cube_index)) + list(range(selected_cube_index + 1, 10)))
#    
#    selected_cube_pos, _ = p.getBasePositionAndOrientation(cubes[selected_cube_index])
#    target_cube_pos, target_cube_ori = p.getBasePositionAndOrientation(cubes[target_cube_index])
#    
#    new_cube_pos = [target_cube_pos[0], target_cube_pos[1], target_cube_pos[2] + cube_size]
#    p.resetBasePositionAndOrientation(cubes[selected_cube_index], new_cube_pos, target_cube_ori)
#
#    # Model intrinsic motivation reward (e.g., height of tower)
#    tower_height = 0
#    for cube_id in cubes:
#        pos, _ = p.getBasePositionAndOrientation(cube_id)
#        tower_height += pos[2]
#    intrinsic_reward = tower_height
#    
#    print(f"Step: {_} - Intrinsic Reward: {intrinsic_reward:.2f}")
#    
#    # Step the simulation
#    p.stepSimulation()
#    time.sleep(0.01)
#
## Disconnect PyBullet simulation
#p.disconnect()
#