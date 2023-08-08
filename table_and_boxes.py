import os
import pybullet as p
import pybullet_data
import math
import time
import random

def random_position():
    return [random.uniform(0, 1), random.uniform(-0.5, 0.5), random.uniform(0.5, 2)]

#def random_cube_id():
#    return random.randint(0, 2)

def assign_position(cube_id):
    object_position, object_orientation = p.getBasePositionAndOrientation(cube_bodies[cube_id]) 
    random_height = random.uniform(0.3,1)
    offset = [0, 0, random_height]
    new_position = [pos1 + off for pos1, off in zip(object_position, offset)]
    return new_position
    

p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", [0,0,-2])

tableUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"),basePosition=[0.5,0,-0.65])

num_cubes = 3
cube_half_extent = 0.1

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
    p.stepSimulation()
    time.sleep(1./240.)
    if(i==60 or i==120):
        p.resetBasePositionAndOrientation(cube_bodies[int(i/60)], assign_position(int(i/60)-1), [0, 0, 0, 1])

p.disconnect()