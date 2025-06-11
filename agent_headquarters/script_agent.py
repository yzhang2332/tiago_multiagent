import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from utils.load_config import load_openai_config
import openai

# Load credentials and IDs
cfg = load_openai_config()
openai.api_key = cfg["openai_api_key"]

script_agent = openai.beta.assistants.create(
    name="ScriptAgent",
    instructions="""
You are ScriptAgent. You have access to task_script.yaml and exp_prompt.yaml. You will receive participants' utterance, then you need to follow the decision style received from exp_prompt.yaml, and decide 
""",
    model="gpt-4o",
    tools=[
        {
            "type": "function",
            "function": {
                "name": "slice_banana",
                "description": "Pick up banana using ArUco marker and place on the cutting board.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "reason": { "type": "string" }
                    },
                    "required": ["reason"]
                }
            }
        }
    ]
)

print("Assistant created:", script_agent.id)
print("⚠️ Copy this ID into config/api_key.yaml under `script_agent_id`")
