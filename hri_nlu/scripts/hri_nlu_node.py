#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from hri_msgs.msg import HRICommand
import json
import re
from llama_cpp import Llama  # CPU-based llama.cpp Python binding

# ------------------------------
# Load the local GGUF model
# ------------------------------
MODEL_PATH = "/home/lucy/ritwik/models/phi-3-mini-4k-instruct-q4.gguf"

# Load once at startup
llm = Llama(model_path=MODEL_PATH)

# ------------------------------
# System prompt for structured JSON
# ------------------------------
SYSTEM_PROMPT = """
You are a RoboCup service robot command parser.

Your task:
Extract the intent and slots from the user command.

Rules:
- Output exactly ONE VALID JSON object
- Do NOT include explanations, comments, markdown, or extra text
- Use null for missing values
- Use only the allowed intents
- Terminate output immediately after closing the JSON object

Allowed intents:
bring_object, go_to, find_person, stop, unknown
a
Required JSON format:
{
  "intent": "bring_object | go_to | find_person | stop | unknown",
  "object": string or null,
  "location": string or null,
  "person": string or null,
  "color": string or null,
  "confidence": number between 0.0 and 1.0
}

"""

# ------------------------------
# LLM call
# ------------------------------
def call_llm(user_text, max_tokens=512):
    prompt = SYSTEM_PROMPT + "\nUser command:\n" + user_text.strip() + "\n"

    # Generate response
    response = llm(
        prompt,
        max_tokens=max_tokens,
        temperature=0.0,
        stop=["}"],  # stop right after the JSON closes
        echo=False
    )

    output_text = response.get("choices")[0]["text"].strip()
    rospy.loginfo(f"Raw LLM output: {output_text}")

    # Remove any prefix before the first {
    json_start = output_text.find("{")
    if json_start == -1:
        rospy.logwarn("No JSON found in LLM output!")
        return None

    json_text = output_text[json_start:] + "}"  # ensure it closes
    try:
        data = json.loads(json_text)
        return data
    except json.JSONDecodeError as e:
        rospy.logwarn(f"Failed to parse JSON: {e}")
        return None

# ------------------------------
# ROS callback
# ------------------------------
def input_callback(msg):
    data = call_llm(msg.data)
    if not data:
        return

    cmd = HRICommand()
    cmd.intent = data.get("intent", "unknown")
    cmd.object = data.get("object") or ""
    cmd.location = data.get("location") or ""
    cmd.person = data.get("person") or ""
    cmd.color = data.get("color") or ""
    cmd.confidence = float(data.get("confidence", 1.0))

    pub.publish(cmd)
    rospy.loginfo(f"[NLU] Published command: {cmd}")

# ------------------------------
# Main ROS node
# ------------------------------
if __name__ == "__main__":
    rospy.init_node("hri_nlu_node")
    pub = rospy.Publisher("/hri/command", HRICommand, queue_size=10)
    rospy.Subscriber("/hri/input_text", String, input_callback)
    rospy.loginfo("NLU node started. Waiting for /hri/input_text...")
    rospy.spin()
