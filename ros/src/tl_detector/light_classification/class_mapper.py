from styx_msgs.msg import TrafficLight

KNOWN_STATES = [
    TrafficLight.RED,
    TrafficLight.YELLOW,
    TrafficLight.GREEN,
    TrafficLight.UNKNOWN]

def map_to(state):
    return [x for x in "{0:04b}".format(KNOWN_STATES.index(state))]

def map_from(index):
    return KNOWN_STATES[index]
