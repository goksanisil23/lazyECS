import json
import random
import pathlib

################# SCENE PARAMETERS #################
SCENE_RADIUS = 10.0
SCENE_CENTER = [0.0, 30.0, 1.0] # [0.0, 24.0, 12.0]
CAMERA_PITCH = -70.0 # degrees # -50

GRID_SIZE_X = 18.0
GRID_SIZE_Z = 18.0
GRID_RESOLUTION = 0.2

APP_STEP_TIME = 0.02

NUM_OBSTACLES = 1 # stuck at 20

################# Actor templates #################
def create_obstacle_box(position, scale):
     
    obst_json = {
            "scale": scale,
            "initial_position": position,
            "initial_rotation": [0,0,0],            
            "shape": "Box",
            "color": [1.0,0.0,0.0,1],
            "tag": "obstacle",
            "body_type": "Static"  
    }

    return obst_json

def create_obstacle_sphere(position, scale):
     
    obst_json = {
            "scale": scale,
            "initial_position": position,
            "initial_rotation": [0,0,0],            
            "shape": "Sphere",
            "color": [1.0,0.0,0.0,1],
            "tag": "obstacle",
            "body_type": "Static"  
    }

    return obst_json


def create_goal(position, scale):
    goal_json = {
            "scale": scale,
            "initial_position": position,
            "initial_rotation": [0,0,0],            
            "shape": "Box",
            "color": [0.0,1.0,0.0,1],
            "tag": "goal",
            "body_type": "Static"   
        }
    return goal_json    

def create_ego(position):

    # return ego_json
    ego_json = {
            "scale": [0.5,0.5,0.5],
            "initial_position": [0,0.5,0],
            "initial_rotation": [0,0,90],
            "body_type": "Dynamic",
            "shape": "Box", #  Dumbbell
            "color": [0,0,1,1],
            "tag": "ego"
    }

    return ego_json    

def create_terrain(position, scale): 
    terrain_json = {
        "scale": scale,
        "initial_position": position,
        "initial_rotation": [0,0,0],        
        "body_type": "Static",
        "shape": "Box",
        "file": ".", # omitted if Shape is not ConcaveMesh
        "heightfield" : { # omitted if Shape is not HeightField
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
    "PhysicsEntities": [create_obstacle_box([2,0.5,2],[1,1,1])] + [create_obstacle_box([4,0.5,4],[1,1,1])] + [create_obstacle_box([6,0.5,6],[1,1,1])] + [create_obstacle_box([8,0.5,8],[1,1,1])] + \
        [create_obstacle_sphere([-4,0.5,-3],[1,1,1])] + [create_obstacle_sphere([6,0.5,-5],[1,1,1])] + [create_obstacle_sphere([0,1,-8],[2,2,2])] + \
        [create_ego([0,0.2,0])] + \
        [create_goal([-2,1,5],[1,2,0.5])] + [create_goal([-4,1,5],[1,2,0.5])] + [create_goal([-6,1,5],[1,2,0.5])],
    "RenderOnlyEntities": [],
    # Build a cage by 1 ground + 4 wall planes 
    "TerrainEntity": [create_terrain(position=[0,0,0],scale=[20.0,0.1,20.0])] +\
        [create_terrain(position=[0,0,-10],scale=[20.0,2.0,0.2])] + [create_terrain(position=[0,0,10],scale=[20.0,2.0,0.2])] +\
            [create_terrain(position=[-10,0,0],scale=[0.2,2.0,20])] + [create_terrain(position=[10,0,0],scale=[0.2,2.0,20])]
}

# Application specific (not related to lazyECS) parameters
application = {
    "GRID_SIZE_X": GRID_SIZE_X,
    "GRID_SIZE_Z": GRID_SIZE_Z,
    "GRID_RESOLUTION": GRID_RESOLUTION,
    "APP_STEP_TIME": APP_STEP_TIME
}

################# Bundle the elements together #################

json_data = {
    "scene" : scene,
    "entities": entities,
    "application": application
}

with open(str(pathlib.Path(__file__).parent.resolve()) + "/../../build/Applications/Raycast/scene.json",'w') as outfile:
    json.dump(json_data, outfile, indent=4)
