import json
import random
import pathlib
################# SCENE PARAMETERS #################
SCENE_RADIUS = 10.0
SCENE_CENTER = [0.0, 35.0, 3.0] # [0.0, 24.0, 12.0]
CAMERA_PITCH = -70.0 # degrees # -50

GRID_SIZE_X = 18.0
GRID_SIZE_Z = 18.0

NUM_OBSTACLES = 50 # stuck at 20

################# Actor templates #################
def create_obstacle(position):
     
    obst_json = {
            "scale": [0.5,1.0,0.5],
            "initial_position": position,
            "initial_rotation": [0,0,0],            
            "shape": "Sphere",
            "color": [1.0,0.0,0.0,1],
            "tag": "obstacle",
            "body_type": "Kinematic"  
    }

    return obst_json

def create_goal(position):
    goal_json = {
            "scale": [0.5,3.0,0.5],
            "initial_position": position,
            "initial_rotation": [0,0,0],            
            "shape": "Sphere",
            "color": [0.0,1.0,0.0,1],
            "tag": "goal",
            "body_type": "Kinematic"   
        }
    return goal_json    

def create_ego(position):
     
    ego_json = {
            "scale": [0.4,0.4,0.4],
            "initial_position": position,
            "initial_rotation": [0,30,0],
            "body_type": "Kinematic",
            "shape": "Sphere",
            "color": [0,0,1,1],
            "tag": "ego"
    }

    return ego_json

def create_terrain(position): 
    terrain_json = {
        "scale": [1.0,1.0,1.0],
        "initial_position": position,
        "initial_rotation": [0,0,0],        
        "body_type": "Static",
        "shape": "ConcaveMesh",
        "file": "/home/goksan/Work/lazyECS/Applications/meshes/walls.obj",
        "heightfield" : {
            "size": 100
        }          
    }
    return terrain_json

def generate_random_position(num_random_positions = 1):
    random_positions = []
    # for _ in random_positions:
    x = random.randint(-GRID_SIZE_X/2.0, GRID_SIZE_X/2.0)
    z = random.randint(-GRID_SIZE_Z/2.0, GRID_SIZE_Z/2.0)
    return [x, 0.5, z]
        # random_positions.append([x, 0.5, z])
    # return random_positions

################# Apply parameters #################
scene = {
    "camera" : {
        "scene_radius": SCENE_RADIUS,
        "center": {"x": SCENE_CENTER[0], "y": SCENE_CENTER[1], "z": SCENE_CENTER[2]},
        "camera_pitch": CAMERA_PITCH
    }
}

entities = {
    "PhysicsEntities": [create_obstacle(generate_random_position()) for _ in range (NUM_OBSTACLES)] + \
        [create_ego(generate_random_position()) for _ in range (1)] + \
        [create_goal(generate_random_position()) for _ in range (1)],
    "RenderOnlyEntities": [],
    "TerrainEntity": create_terrain([0,0,0])
}

################# Bundle the elements together #################

json_data = {
    "scene" : scene,
    "entities": entities
}

with open(str(pathlib.Path(__file__).parent.resolve()) + "/../../build/Applications/PotentialField/scene.json",'w') as outfile:
    json.dump(json_data, outfile, indent=4)



