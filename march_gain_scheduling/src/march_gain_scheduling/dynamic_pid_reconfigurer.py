from dynamic_reconfigure.client import Client
import rospy

from march_shared_resources.msg import GaitNameActionGoal


class DynamicPIDReconfigurer:
    def __init__(self, gait_name=None, joint_list=None):
        self._gait_name = gait_name
        self._joint_list = joint_list
        self._clients = []
        for i in range(len(self._joint_list)):
            self._clients.append(Client('/march/controller/trajectory/gains/' + self._joint_list[i], timeout=30))
        rospy.Subscriber('/march/gait/schedule/goal', GaitNameActionGoal, callback=self.gait_selection_callback)

    def gait_selection_callback(self, data):
        rospy.logdebug('This is the gait name: %s', data.goal.current_subgait.gait_type)
        if self._gait_name != data.goal.name:
            rospy.logdebug('The selected gait: {0} is not the same as the previous gait: {1}'.format(
                data.goal.current_subgait.gait_type, self._gait_name))
            self._gait_name = data.goal.current_subgait.gait_type
            self.client_update()

    def client_update(self):
        for i in range(len(self._joint_list)):
            p_value, i_value, d_value = self.look_up_table(i)
            if p_value is not None:
                self._clients[i].update_configuration({'p': p_value,
                                                       'i': i_value,
                                                       'd': d_value})
                rospy.logdebug('Config set to {0}, {1}, {2}'.format(p_value, i_value, d_value))

    # Method that pulls the PID values from the gains_per_gait_type.yaml config file
    def look_up_table(self, i):
        if rospy.has_param('~gait_types/' + self._gait_name + '_gains/'):
            gains = rospy.get_param('~gait_types/' + self._gait_name + '_gains/' + self._joint_list[i])
            return gains['p'], gains['i'], gains['d']
        else:
            return None, None, None
