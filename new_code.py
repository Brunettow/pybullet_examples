import os
import pybullet as p
import pybullet_data
import math
import time
import random
import numpy as np

epsilon = 0.1
max_height = 0.05

def random_position():  # for getting random positions in the beginning
    return [random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(0.5, 2)]

def assign_position(cube_id):
    object_position, object_orientation = p.getBasePositionAndOrientation(cube_id) 
    random_height = random.uniform(max_height,1)
    offset = [0, 0, random_height]
    new_position = [pos1 + off for pos1, off in zip(object_position, offset)]
    return new_position
    
p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", [0,0,0])

num_cubes = 10
cube_half_extent = 0.05

cube_bodies = []
for _ in range(num_cubes):
    cube_start_pos = random_position()
    cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    cube_collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_half_extent, cube_half_extent, cube_half_extent])
    cube_body_id = p.createMultiBody(baseMass=1.0,
                                     baseCollisionShapeIndex=cube_collision_shape_id,
                                     basePosition=cube_start_pos,
                                     baseOrientation=cube_start_orientation)
    cube_bodies.append(cube_body_id)

random.shuffle(cube_bodies)

tower_cubes = []

for i in range (10000):
    for y in range(500):
        p.stepSimulation()
        time.sleep(2.0/480)

    r = random.random()
    
    if(r < epsilon): # choose random action
        destination_cube_id = random.choice(cube_bodies)
        target_cube_id = random.choice(cube_bodies)
  
    else: # choose greedy action
        destination_cube_id = -1
        if tower_cubes:
            destination_cube_id = random.choice(tower_cubes)
        else:
            destination_cube_id = random.choice(cube_bodies)
        destination_position, _ = p.getBasePositionAndOrientation(destination_cube_id)
        target_cube_id = random.choice(cube_bodies)
        target_position, _ = p.getBasePositionAndOrientation(target_cube_id)
    
        while (target_position[0] == destination_position[0] and target_position[1] == destination_position[1]) :
            target_cube_id = random.choice(cube_bodies)
            target_position, _ = p.getBasePositionAndOrientation(target_cube_id)
        
    _ , reference_orientation = p.getBasePositionAndOrientation(destination_cube_id)
    p.resetBasePositionAndOrientation(target_cube_id, assign_position(destination_cube_id), reference_orientation)
    curr_object_position, _ = p.getBasePositionAndOrientation(target_cube_id)
    new_height = curr_object_position[2] # + 0.05 # or 0.1?
    
    if(max_height == new_height):
        tower_cubes.append(destination_cube_id)

    elif(max_height < new_height):
        max_height = new_height
        tower_cubes = [destination_cube_id]
        print(f"new max height: {max_height}, simulation_time: {i}")

p.disconnect()