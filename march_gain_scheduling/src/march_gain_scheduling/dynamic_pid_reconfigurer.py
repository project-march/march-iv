from dynamic_reconfigure.client import Client
import rospy

from march_shared_resources.msg import GaitActionGoal

from .one_step_linear_interpolation import interpolate


class DynamicPIDReconfigurer:
    def __init__(self, gait_type='walk_like', joint_list=None, max_time_step=0.1):
        self._gait_type = gait_type
        self._joint_list = joint_list
        self._max_time_step = max_time_step
        self.current_gains = [self.look_up_table(i) for i in range(len(self._joint_list))]
        self.interpolation_done = True
        self.last_update_time = 0
        self._clients = []
        for i in range(len(self._joint_list)):
            self._clients.append(Client('/march/controller/trajectory/gains/' + self._joint_list[i], timeout=30))
        rospy.Subscriber('/march/gait/schedule/goal', GaitActionGoal, callback=self.gait_selection_callback)
        self._linearize = rospy.get_param('~linearize_gain_scheduling')

    def gait_selection_callback(self, data):
        rospy.logdebug('This is the gait name: %s', data.goal.current_subgait.gait_type)
        if self._gait_type != data.goal.current_subgait.gait_type or not self.interpolation_done:
            rospy.logdebug('The selected gait: {0} is not the same as the previous gait: {1}'.format(
                data.goal.current_subgait.gait_type, self._gait_type))
            self._gait_type = data.goal.current_subgait.gait_type
            self.interpolation_done = False
            rate = rospy.Rate(10)
            while not self.interpolation_done:
                rate.sleep()
                self.client_update()

    def client_update(self):
        rospy.logdebug('self.linearize is: {0}'.format(self._linearize))
        if self._linearize:
            self.interpolation_done = True
            for i in range(len(self._joint_list)):
                needed_gains = self.look_up_table(i)
                gradient = rospy.get_param('~linear_slope')
                current_time = rospy.get_time()
                time_interval = current_time - self.last_update_time
                self.current_gains[i] = interpolate(self.current_gains[i], needed_gains, gradient, time_interval)
                if self.current_gains[i] != self.needed_gains:
                    self.interpolation_done = False
                self._clients[i].update_configuration({'p': self.current_gains[i][0],
                                                       'i': self.current_gains[i][1],
                                                       'd': self.current_gains[i][2]})
                rospy.logdebug('Config set to {0}, {1}, {2}'.format(self.current_gains[i][0],
                                                                    self.current_gains[i][1],
                                                                    self.current_gains[i][2]))
                self.last_update_time = current_time
        else:
            self.interpolation_done = True
            for i in range(len(self._joint_list)):
                p_value, i_value, d_value = self.look_up_table(i)
                if p_value is not None:
                    self._clients[i].update_configuration({'p': p_value,
                                                           'i': i_value,
                                                           'd': d_value})
                    rospy.logdebug('nonlinear Config set to {0}, {1}, {2}'.format(p_value, i_value, d_value))

    # Method that pulls the PID values from the gains_per_gait_type.yaml config file
    def look_up_table(self, joint_index):
        if rospy.has_param('~gait_types/' + self._gait_type):
            gains = rospy.get_param('~gait_types/' + self._gait_type + '/' + self._joint_list[joint_index])
            return [gains['p'], gains['i'], gains['d']]
        else:
            return [None, None, None]
