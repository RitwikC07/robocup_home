#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from hri_msgs.msg import HRICommand
import subprocess
import json
import rospy
from std_msgs.msg import String
from hri_msgs.msg import HRICommand
import subprocess
import tempfile
import json
import re
import os

LLAMA_BIN = "/home/ritwik/llama.cpp/build/bin/llama-cli"
MODEL = "/home/ritwik/models/phi-3-mini-instruct-q4.gguf"

SYSTEM_PROMPT = """
You are a RoboCup service robot command parser.
Output ONLY valid JSON.
Do NOT add explanations, notes, or markdown.
Allowed intents: bring_object, go_to, find_person, stop, unknown.
Use this schema exactly:
Schema:
{
  "intent": string,
  "object": string or null,
  "location": string or null,
  "person": string or null,
  "color": string or null,
  "confidence": float (0.0 to 1.0)
}
"""

# def call_llm(text):
#     prompt = SYSTEM_PROMPT + "\nUser: " + text + "\nAssistant:"
#     result = subprocess.run(
#         [LLAMA_BIN, "-m", MODEL, "-p", prompt, "--temp", "0"],
#         capture_output=True, text=True
#     )
#     return result.stdout.strip()

# def call_llm(text):
#     prompt = SYSTEM_PROMPT + "\nUser: " + text + "\nAssistant:"
#     result = subprocess.run(
#         [
#             LLAMA_BIN,
#             "-m", MODEL,
#             "--instruct",
#             "-p", prompt,
#             "--temp", "0",
#             "--exit-on-eos"
#         ],
#         capture_output=True,
#         text=True
#     )
#     return result.stdout.strip()


# def callback(msg):
#     try:
#         data = json.loads(call_llm(msg.data))
#         cmd = HRICommand()
#         cmd.intent = data.get("intent", "unknown")
#         cmd.object = data.get("object") or ""
#         cmd.location = data.get("location") or ""
#         cmd.person = data.get("person") or ""
#         cmd.color = data.get("color") or ""
#         cmd.confidence = float(data.get("confidence", 1.0))
#         pub.publish(cmd)
#         rospy.loginfo(f"NLU parsed: {cmd}")
#     except Exception as e:
#         rospy.logwarn(f"NLU failed: {e}")

# def callback(msg):
#     try:
#         raw = call_llm(msg.data)
#         import re
#         import json
#         match = re.search(r"\{.*\}", raw, re.DOTALL)
#         if not match:
#             rospy.logwarn(f"NLU failed: no JSON found in LLM output: {raw}")
#             return
#         data = json.loads(match.group(0))
        
#         cmd = HRICommand()
#         cmd.intent = data.get("intent", "unknown")
#         cmd.object = data.get("object") or ""
#         cmd.location = data.get("location") or ""
#         cmd.person = data.get("person") or ""
#         cmd.color = data.get("color") or ""
#         cmd.confidence = float(data.get("confidence", 1.0))
#         pub.publish(cmd)
#         rospy.loginfo(f"Published command: {cmd}")
#     except Exception as e:
#         rospy.logwarn(f"NLU failed: {e}")


# rospy.init_node("hri_nlu_node")
# pub = rospy.Publisher("/hri/command", HRICommand, queue_size=10)
# rospy.Subscriber("/hri/input_text", String, callback)
# rospy.spin()

# === FUNCTION TO CALL LLM ===
def call_llm(user_text):
    # Create temp file with the prompt
    with tempfile.NamedTemporaryFile(mode='w+', delete=False) as f:
        f.write(SYSTEM_PROMPT + user_text + "\nAssistant:")
        prompt_file = f.name

    try:
        # Call llama-cli using the temp file
        result = subprocess.run(
            [LLAMA_BIN, "-m", MODEL, "-f", prompt_file, "-n", "256"],
            capture_output=True,
            text=True,
        )
        raw_output = result.stdout.strip()
    finally:
        # Clean up temp file
        os.remove(prompt_file)

    # Extract JSON safely
    match = re.search(r"\{.*?\}", raw_output, re.DOTALL)  # non-greedy
    if not match:
        rospy.logwarn(f"NLU output did not contain valid JSON:\n{raw_output}")
        return None

    try:
        data = json.loads(match.group(0))
    except json.JSONDecodeError as e:
        rospy.logwarn(f"Failed to parse JSON: {e}\n{raw_output}")
        return None

    return data

# === ROS CALLBACK ===
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

# === MAIN NODE ===
if __name__ == "__main__":
    rospy.init_node("hri_nlu_node")
    pub = rospy.Publisher("/hri/command", HRICommand, queue_size=10)
    rospy.Subscriber("/hri/input_text", String, input_callback)
    rospy.loginfo("NLU node started. Waiting for /hri/input_text...")
    rospy.spin()
