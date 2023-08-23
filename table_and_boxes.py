import os
import pybullet as p
import pybullet_data
import math
import time
import random

def random_position():
    return [random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(0.5, 2)]

#def random_cube_id():
#    return random.randint(0, 2)

def assign_position(cube_id):
    object_position, object_orientation = p.getBasePositionAndOrientation(cube_id) 
    random_height = random.uniform(0.3,1)
    offset = [0, 0, random_height]
    new_position = [pos1 + off for pos1, off in zip(object_position, offset)]
    return new_position
    

p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", [0,0,0])

#tableUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"),basePosition=[0,0,0])

num_cubes = 5
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
position_changed = [False]*(len(cube_bodies)-1)

for i in range (10000):
    for y in range(500):
        p.stepSimulation()
        time.sleep(2.0/480)
    random_cube_id = random.choice(cube_bodies)
    p.resetBasePositionAndOrientation(random.choice(cube_bodies), assign_position(random_cube_id), [0, 0, 0, 1])
    curr_object_position, curr_object_orientation = p.getBasePositionAndOrientation(random_cube_id)
    print(f"Cube {random_cube_id} location:", curr_object_position)


p.disconnect()