#!/usr/bin/env python3
import rospy
import threading
from hri_msgs.msg import HRICommand
from llama_cpp import Llama

# ==============================
# Load LLM
# ==============================
MODEL_PATH = "/home/lucy/ritwik/models/phi-3-mini-4k-instruct-q4.gguf"
llm = Llama(model_path=MODEL_PATH)

# ==============================
# Tool Layer (MCP-style)
# ==============================


class RobotTools:

    def go_to(self, location):
        rospy.loginfo(f"[Planner] Calling go_to({location})")

        # TODO: Replace with real ROS action/service
        success = True

        return {"status": "success" if success else "failure"}

    def detect_object(self, obj, color):
        rospy.loginfo(f"[Planner] detect_object({obj}, {color})")

        # TODO: Replace with real detection/grasp action
        success = True

        return {"status": "success" if success else "failure"}

    def perform_handover(self):
        rospy.loginfo("[Planner] perform_handover()")

        # TODO: Replace with real handover action
        success = True

        return {"status": "success" if success else "failure"}


tools = RobotTools()

# ==============================
# Planner Prompt Builder
# ==============================


def build_prompt(task):

    return f"""
You are a robot task planner.

Task:
Intent: {task.intent}
Object: {task.object}
Location: {task.location}
Color: {task.color}

Available tools:
- go_to(location)
- detect_object(object, color)
- perform_handover()

Rules:
- Always wait for tool result before next step.
- Only call one tool at a time.
- If a tool fails, retry once.
- End after task is complete.

Respond ONLY with:
CALL <tool_name> <json_arguments>

Example:
CALL go_to {{"location": "kitchen"}}
"""


# ==============================
# LLM Execution Loop
# ==============================


def execute_task(task):

    prompt = build_prompt(task)
    done = False
    retries = 0

    while not done and not rospy.is_shutdown():

        response = llm(prompt, max_tokens=200, temperature=0.0)

        text = response["choices"][0]["text"].strip()
        rospy.loginfo(f"[Planner] LLM: {text}")

        if not text.startswith("CALL"):
            rospy.loginfo("[Planner] Task complete.")
            break

        parts = text.split(" ", 2)
        tool_name = parts[1]

        import json

        args = json.loads(parts[2])

        result = call_tool(tool_name, args)

        if result["status"] == "failure":
            retries += 1
            if retries > 1:
                rospy.logwarn("[Planner] Tool failed twice. Aborting.")
                break
        else:
            retries = 0

        prompt += f"\nTool result: {result}\n"


# ==============================
# Tool Dispatcher
# ==============================


def call_tool(name, args):

    ALLOWED_LOCATIONS = ["kitchen", "lab", "office", "user"]

    if name == "go_to":
        location = args.get("location")
        if location not in ALLOWED_LOCATIONS:
            return {"status": "failure", "reason": "invalid location"}
        return tools.go_to(location)

    elif name == "detect_object":
        return tools.detect_object(args.get("object"), args.get("color"))

    elif name == "perform_handover":
        return tools.perform_handover()

    return {"status": "failure", "reason": "unknown tool"}


# ==============================
# ROS Callback
# ==============================


def final_command_callback(msg):
    rospy.loginfo(f"[Planner] Received final command: {msg}")

    thread = threading.Thread(target=execute_task, args=(msg,))
    thread.start()


# ==============================
# Main
# ==============================

if __name__ == "__main__":
    rospy.init_node("robot_planner_node")

    rospy.Subscriber("/hri/command_final", HRICommand, final_command_callback)

    rospy.loginfo("[Planner] Robot planner node started.")
    rospy.spin()
