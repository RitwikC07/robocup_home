#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from hri_msgs.msg import HRICommand

# --- REQUIRED SLOTS PER INTENT ---
REQUIRED_SLOTS = {
    "bring_object": ["object", "location"],
    "go_to": ["location"],
    "find_person": ["person", "location"],
    "stop": [],
    "unknown": []
}

# --- CONTEXT STORE ---
# Keeps track of the current dialogue state
current_context = {}

# --- PUBLISHER ---
pub_response = None

# --- CALLBACK ---
def command_callback(cmd: HRICommand):
    global current_context

    # Update context with latest command
    current_context['intent'] = cmd.intent
    current_context['object'] = cmd.object
    current_context['location'] = cmd.location
    current_context['person'] = cmd.person
    current_context['color'] = cmd.color
    current_context['confidence'] = cmd.confidence

    rospy.loginfo(f"[Dialogue] Received command: {cmd}")

    # Check for missing slots
    missing = []
    for slot in REQUIRED_SLOTS.get(cmd.intent, []):
        if not current_context.get(slot):
            missing.append(slot)

    if missing:
        # Ask user for missing information
        response_text = "Please tell me the " + ", ".join(missing) + "."
        pub_response.publish(response_text)
        rospy.loginfo(f"[Dialogue] Asking for missing info: {response_text}")
    else:
        # All info available → ready to execute
        response_text = f"All information received. Executing {cmd.intent}."
        pub_response.publish(response_text)
        rospy.loginfo(f"[Dialogue] {response_text}")
        # Optionally, reset context after execution
        current_context.clear()

# --- MAIN ---
if __name__ == "__main__":
    rospy.init_node("hri_dialogue_node")

    # Publisher to send responses back to user
    pub_response = rospy.Publisher("/hri/response", String, queue_size=10)

    # Subscriber to receive parsed commands from NLU
    rospy.Subscriber("/hri/command", HRICommand, command_callback)

    rospy.loginfo("[Dialogue] Dialogue manager started. Waiting for commands...")
    rospy.spin()



# #!/usr/bin/env python3
# import rospy
# from hri_msgs.msg import HRICommand
# from std_msgs.msg import String

# current = {
#     "intent": None,
#     "object": None,
#     "location": None,
#     "color": None
# }

# REQUIRED = {
#     "bring_object": ["object", "location"]
# }

# def update(cmd):
#     if cmd.intent:
#         current["intent"] = cmd.intent
#     if cmd.object:
#         current["object"] = cmd.object
#     if cmd.location:
#         current["location"] = cmd.location
#     if cmd.color:
#         current["color"] = cmd.color

# def callback(cmd):
#     rospy.loginfo(f"Received command: {cmd}")
#     update(cmd)
#     intent = current["intent"]

#     if intent not in REQUIRED:
#         pub.publish("I am not sure what you want me to do.")
#         return

#     for slot in REQUIRED[intent]:
#         if not current[slot]:
#             pub.publish(f"Please tell me the {slot}.")
#             return

#     pub.publish("Okay, I have all the information. Executing now.")

# rospy.init_node("dialogue_manager")
# pub = rospy.Publisher("/hri/response", String, queue_size=10)
# rospy.Subscriber("/hri/command", HRICommand, callback)
# rospy.spin()