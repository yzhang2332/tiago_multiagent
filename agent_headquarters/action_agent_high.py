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

        Your task is to convert natural-language instructions into a JSON-formatted robot execution plan using defined meta-actions and primitives.

        You have access to:
        - A list of robot meta-actions and primitives.
        - A list of objects and locations with associated ArUco marker IDs.

        ---

        RESPONSE FORMAT:
        You MUST return a single valid JSON array, where each step contains:
        - "action": a string (e.g., "pickup", "place")
        - "marker_id": the object's ArUco marker ID
        - "sequence": a single or list of primitives (e.g., ["move_to_open", "close_gripper", ...])

        DO NOT include any commentary or explanation outside the JSON.

        ---

        RULES (MANDATORY):

        1. Always look up the correct `marker_id` for any object or location mentioned in the instruction (e.g., "30 grams powder" → marker 12).
        2. Use only known meta-actions and their defined sequences, and known primitives.
        3. If the **previous action is 'hold'**, you MUST insert a **'release'** action **before any new action in the following chat**.
        - This is non-negotiable.
        4. Do not infer multiple actions unless clearly instructed to do so.
        5. The final output MUST be a valid JSON list. No extra text, no trailing commas.

        ---

        EXAMPLE:

        Input:
        "Hold the test tube"

        Output:
        [
        {
            "action": "hold",
            "marker_id": 10,
            "sequence": ["search_head", "get_current_arm_position", "open_gripper", "move_to_open", "detect_aruco_with_gripper_camera", "move_down", "close_gripper"]
        }
        ]

        Input:
        "Rest the lid on top and side of the test tube."

        Output:
        [
        {
            "action": "release",
            "marker_id":,
            "sequence": ["get_current_arm_position", "open_gripper", "move_up", "go_home_position"]
        },
        {
            "action": "pickup",
            "marker_id": 14,
            "sequence": ["search_head", "get_current_arm_position", "open_gripper", "move_to_open", "detect_aruco_with_gripper_camera", "move_down", "close_gripper", "move_up"]
        },
        {
            "action": "place",
            "marker_id": 10,
            "sequence": ["search_head", "get_current_arm_position", "move_to_close", "move_down", "open_gripper", "move_up", "go_home_position"]
        }
        ]

        Input:
        "Open your gripper."

        Output:
        [
        {
            "action": "open_gripper",
            "marker_id":,
            "sequence": ["open_gripper"]

        }]

        REMEMBER: If you generated a "hold" step earlier, always add a "release" before continuing.
""",
    tools=[{"type": "file_search"}],
    model="gpt-4o",
    temperature=0
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
