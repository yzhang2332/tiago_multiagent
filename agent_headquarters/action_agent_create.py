# import sys
# import os
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# from utils.load_config import load_openai_config
# import openai

# # Load credentials and IDs
# cfg = load_openai_config()
# openai.api_key = cfg["openai_api_key"]

# action_agent = openai.beta.assistants.create(
#     name="ActionAgent",
#     instructions="""
# You are ActionAgent inside a robot system. You receive natural-language action instructions from another assistant.

# Your job is to classify each instruction into a known robot action keyword. These keywords represent internal robot routines.

# Your response MUST be only one of the following exact strings (no punctuation or explanation):
# "pickup", "place", "hold", "rest", "open_gripper", "close_gripper", "move_up", "search_head"

# Examples:
# - Input: "Bring me the container with 30 grams" → "pickup"
# - Input: "Place it on the tray" → "place"
# - Input: "Open the gripper" → "open_gripper"
# - Input: "Stabilise it please" → "hold"
# - Input: "Return to your resting pose" → "rest"

# Important rules:
# 1. Always respond with only the action keyword, in lowercase.
# 2. Do NOT explain. Do NOT use punctuation.
# 3. If the instruction is ambiguous, choose the most likely one.
# 4. If none apply, choose the best matching primitive from the list.

# Respond ONLY with the action name.
# """,
#     model="gpt-4o"
# )

# print("ActionAgent created:", action_agent.id)

import os
import sys
import openai

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from utils.load_config import load_openai_config

# Load API key from config
cfg = load_openai_config()
client = openai.OpenAI(api_key=cfg["openai_api_key"])

# Upload files for assistant use
behavior_file_path = "../config/behavior.json"
aruco_file_path = "../config/object_aruco.json"

# Upload to vector store
vector_store = client.beta.vector_stores.create(name="action_context")

client.beta.vector_stores.file_batches.upload_and_poll(
    vector_store_id=vector_store.id,
    files=[
        open(behavior_file_path, "rb"),
        open(aruco_file_path, "rb")
    ]
)

# Create the assistant (without attaching the vector store yet)
assistant = client.beta.assistants.create(
    name="ActionAgent",
    instructions="""
You are ActionAgent inside a robot system.

Your job is to convert a natural-language instruction like:
"Pick up the 30 grams powder and place it in the middle of the table"
into a full robot execution plan.

You are provided with:
- A list of known behaviors (meta-actions and their primitives)
- A list of known ArUco markers and their object/semantic labels

Your response must be a valid JSON list of steps, each with:
- action (e.g. "pickup", "place")
- marker_id (e.g. 23)
- sequence (a list of primitives)

For example:
[
  {
    "action": "pickup",
    "marker_id": 23,
    "sequence": ["coarse_localise_target", "get_current_arm_position", "open_gripper", "move_to_open", "close_gripper", "move_up"]
  },
  {
    "action": "place",
    "marker_id": 57,
    "sequence": ["coarse_localise_target", "get_current_arm_position", "move_to_close", "open_gripper", "move_up"]
  }
]

Your output must only be the JSON list. No explanation.
""",
    tools=[{"type": "file_search"}],
    model="gpt-4o"
)

# Attach the vector store to the assistant
client.beta.assistants.update(
    assistant_id=assistant.id,
    tool_resources={
        "file_search": {
            "vector_store_ids": [vector_store.id]
        }
    }
)

print("ActionAgent created:", assistant.id)
