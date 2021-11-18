import yaml
import matplotlib.pyplot as plt

def return_track():
    '''
    
    '''
    yaml_file = open("src/perception/ml_training/ml_training/sub_module/yaml_track.txt")

    parsed = dict(yaml.load(yaml_file, Loader=yaml.FullLoader))

    cones_locs = parsed["cones"]

    cones = list()
    for cone_loc in parsed["cones"]:
        cone = list(cone_loc.values())
        cones.append(cone)

    return cones

