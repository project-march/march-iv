
import rospy
import yaml

from march_shared_classes.gait.subgait import Subgait


class SubgaitSelector(Subgait):
    def __init__(self, joints, duration):
        Subgait.__init__(self, joints, duration)
