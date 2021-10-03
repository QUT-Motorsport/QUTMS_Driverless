import yaml
yaml_file = open("src/perception/ml_training/ml_training/test.txt")

parsed = dict(yaml.load(yaml_file, Loader=yaml.FullLoader))

cones_locs = parsed["cones"]

cones = list()
for cone_loc in parsed["cones"]:
    print("\n", cone_loc)

    cone = list(cone_loc.values())
    print(cone)
    cones.append(cone)

with open("src/perception/ml_training/ml_training/track.txt", 'w') as output:
    output.write(str(cones) + '\n')