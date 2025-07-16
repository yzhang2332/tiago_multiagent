import json
import yaml
import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))

# Define the config folder path
config_folder = os.path.join(script_dir, "../config")
config_folder = os.path.abspath(config_folder)

# Ensure the config folder exists
if not os.path.exists(config_folder):
    print(f"Error: The folder '{config_folder}' does not exist.")
    exit(1)

# Get all .json files in the config folder
json_files = [f for f in os.listdir(config_folder) if f.endswith('.json')]
# json_files = ["task_high.json"]

if not json_files:
    print(f"No .json files found in the '{config_folder}' folder.")
    exit(1)

# Process each .json file
for json_file in json_files:
    json_path = os.path.join(config_folder, json_file)
    
    # Read the JSON file
    try:
        with open(json_path, 'r') as jf:
            json_data = json.load(jf)
        
        # Convert to YAML
        yaml_data = yaml.dump(json_data, allow_unicode=True, default_flow_style=False)
        
        # Create output YAML file path (same name, different extension)
        yaml_file = os.path.splitext(json_file)[0] + '.yaml'
        yaml_path = os.path.join(config_folder, yaml_file)
        
        # Write to YAML file
        with open(yaml_path, 'w') as yf:
            yf.write(yaml_data)
        
        print(f"Successfully converted {json_file} to {yaml_file}")
    
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON in {json_file}: {e}")
    except Exception as e:
        print(f"Error processing {json_file}: {e}")