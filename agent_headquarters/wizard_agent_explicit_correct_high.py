import os
import sys
import openai

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from utils.load_config import load_openai_config

cfg = load_openai_config()
client = openai.OpenAI(api_key=cfg["openai_api_key"])

vector_store = client.beta.vector_stores.create(name="wizard_context_store")

client.beta.vector_stores.file_batches.upload_and_poll(
    vector_store_id=vector_store.id,
    files=[
        open("../config/high_task.json", "rb"),
        open("../config/behavior_high.json", "rb"),
        open("../config/object_aruco_high.json", "rb")
    ]
)

wizard_agent = client.beta.assistants.create(
    name="WizardAgentExplicitCorrectHigh",
    instructions="""
You are a WizardAgent embedded in a human-robot collaboration framework. Your role is to monitor task progress and system state using contextual files and ROS system messages.

You have access to:
- A task script (high_task.json) that defines all expected high-level task steps.
- A behaviour specification (behavior_high.json) describing robot actions and capabilities.
- A mapping of object names to ArUco marker IDs (object_aruco_high.json).

Your job is to:
1. Analyse the **recent ROS messages** (e.g. execution_status, agent status, action_instruction).
2. Compare the current execution state with the **planned steps** from high_task.json.
3. Identify the **current step**, **what has been completed**, and **what is expected next**.
4. Generate a **brief summary of the current task state**, written as several short, clear, and concise sentences.

Your output should:
- Mention what part of the task is currently being executed.
- Include if the robot is waiting for a command, performing an action, or completed a step.
- Avoid speculation or planning beyond the available information.
- Use polite, clear English suitable for real-time robot feedback.

---
OUTPUT FORMAT (Required):

Task Summary:
- <Sentence 1>
- <Sentence 2>
- ...

Do NOT return JSON. Do NOT include explanations about how you reasoned. Be direct and informative.
""",
    tools=[{"type": "file_search"}],
    model="gpt-4o",
    temperature=0
)

