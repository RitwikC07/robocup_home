#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from hri_msgs.msg import HRICommand

# ==============================
# Required slots per intent
# ==============================
REQUIRED_SLOTS = {
    "bring_object": ["object", "location"],
    "go_to": ["location"],
    "find_person": ["person"],
    "stop": [],
    "unknown": []
}

# ==============================
# Clarification questions
# ==============================
CLARIFICATION_QUESTIONS = {
    "object": "Which object do you mean?",
    "location": "Where should I get it from?",
    "person": "Who are you referring to?",
    "color": "What color is it?"
}

# ==============================
# Dialogue state (GLOBAL)
# ==============================
dialogue_active = False
locked_intent = None
current_command = None

response_pub = None
final_cmd_pub = None

# ==============================
# Helpers
# ==============================
def hri_to_dict(msg: HRICommand):
    return {
        "intent": msg.intent,
        "object": msg.object if msg.object else None,
        "location": msg.location if msg.location else None,
        "person": msg.person if msg.person else None,
        "color": msg.color if msg.color else None,
        "confidence": msg.confidence
    }

def dict_to_hri(data: dict):
    msg = HRICommand()
    msg.intent = data.get("intent", "unknown")
    msg.object = data.get("object") or ""
    msg.location = data.get("location") or ""
    msg.person = data.get("person") or ""
    msg.color = data.get("color") or ""
    msg.confidence = float(data.get("confidence", 1.0))
    return msg

# ==============================
# Slot logic
# ==============================
def get_missing_slots():
    intent = current_command["intent"]
    required = REQUIRED_SLOTS.get(intent, [])

    missing = []
    for slot in required:
        if current_command.get(slot) is None:
            missing.append(slot)

    return missing

def merge_slots(new_cmd):
    global current_command

    for slot in ["object", "location", "person", "color"]:
        if current_command.get(slot) is None and new_cmd.get(slot):
            current_command[slot] = new_cmd[slot]
            rospy.loginfo(f"[DM] Filled slot '{slot}' → '{new_cmd[slot]}'")

# ==============================
# Dialogue flow
# ==============================
def ask_clarification(slot):
    question = CLARIFICATION_QUESTIONS.get(slot, "Can you clarify?")
    rospy.loginfo(f"[DM] Asking: {question}")
    response_pub.publish(String(question))

def finish_dialogue():
    global dialogue_active, locked_intent, current_command

    final_msg = dict_to_hri(current_command)
    final_cmd_pub.publish(final_msg)

    rospy.loginfo(f"[DM] Final command published: {final_msg}")

    # Reset state
    dialogue_active = False
    locked_intent = None
    current_command = None

def process_state():
    missing = get_missing_slots()

    if not missing:
        rospy.loginfo("[DM] All required slots filled")
        finish_dialogue()
    else:
        ask_clarification(missing[0])

# ==============================
# ROS callback
# ==============================
def command_callback(msg: HRICommand):
    global dialogue_active, locked_intent, current_command

    rospy.loginfo(f"[DM] Received: {msg}")

    incoming = hri_to_dict(msg)

    # Start new dialogue
    if not dialogue_active:
        dialogue_active = True
        locked_intent = incoming["intent"]
        current_command = incoming

        rospy.loginfo(f"[DM] New dialogue started | intent locked: {locked_intent}")
        process_state()
        return

    # Ongoing dialogue
    if incoming["intent"] != locked_intent:
        rospy.logwarn(
            f"[DM] Ignoring intent change {incoming['intent']} (locked: {locked_intent})"
        )

    merge_slots(incoming)
    process_state()

# ==============================
# Main
# ==============================
if __name__ == "__main__":
    rospy.init_node("hri_dialogue_manager")

    response_pub = rospy.Publisher("/hri/response", String, queue_size=10)
    final_cmd_pub = rospy.Publisher("/hri/command_final", HRICommand, queue_size=10)

    rospy.Subscriber("/hri/command", HRICommand, command_callback)

    rospy.loginfo("[DM] Dialogue Manager started (no classes)")
    rospy.spin()