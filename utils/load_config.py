import yaml
import os

def load_openai_config(path="config/api_key.yaml"):
    abs_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", path))
    with open(abs_path, "r") as f:
        return yaml.safe_load(f)
