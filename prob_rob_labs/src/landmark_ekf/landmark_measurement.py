import json

# Specify the path to your JSON file
file_path = "/run/host/workdir/local_oc2356/ros2_ws/landmark_map.json"

# Open the JSON file in read mode ('r')
with open(file_path, 'r') as file:
# Use json.load() to parse the JSON data from the file
    data = json.load(file)


