from dynamic_reconfigure.client import Client
import rospy

from march_shared_resources.msg import GaitActionGoal

from .one_step_linear_interpolation import interpolate


class DynamicPIDReconfigurer:
    def __init__(self, joint_list=None, max_time_step=0.1):
        self._gait_type = None
        self._joint_list = joint_list
        self._max_time_step = max_time_step
        self.interpolation_done = True
        self.last_update_time = None
        self._clients = []
        for i in range(len(self._joint_list)):
            self._clients.append(Client('/march/controller/trajectory/gains/' + self._joint_list[i], timeout=30))
        rospy.Subscriber('/march/gait/schedule/goal', GaitActionGoal, callback=self.gait_selection_callback)
        self._linearize = rospy.get_param('~linearize_gain_scheduling')

    def gait_selection_callback(self, data):
        rospy.logdebug('This is the gait type: %s', data.goal.current_subgait.gait_type)
        new_gait_type = data.goal.current_subgait.gait_type
        if new_gait_type is None or new_gait_type == '':
            new_gait_type = 'walk_like'
            rospy.logwarn('The gait has no gait type, default is set to walk_like')

        if self._gait_type != new_gait_type or not self.interpolation_done:
            rospy.logdebug('The selected gait: {0} is not the same as the previous gait: {1}'.format(
                new_gait_type, self._gait_type))

            self.load_current_gains()
            self._gait_type = new_gait_type
            self.interpolation_done = False
            rate = rospy.Rate(10)
            self.last_update_time = rospy.get_time()
            while not self.interpolation_done:
                rate.sleep()
                self.client_update()

    def client_update(self):
        rospy.logdebug('self.linearize is: {0}'.format(self._linearize))
        self.interpolation_done = True

        for i in range(len(self._joint_list)):
            if self._linearize:
                needed_gains = self.look_up_table(i)
                gradient = rospy.get_param('~linear_slope')
                current_time = rospy.get_time()
                time_interval = current_time - self.last_update_time

                self.current_gains[i] = interpolate(self.current_gains[i], needed_gains, gradient, time_interval)
                if self.current_gains[i] != needed_gains:
                    self.interpolation_done = False
                self.last_update_time = current_time
            else:
                self.current_gains[i] = self.look_up_table(i)

            self._clients[i].update_configuration({'p': self.current_gains[i][0],
                                                   'i': self.current_gains[i][1],
                                                   'd': self.current_gains[i][2]})
            rospy.logdebug('Config set to {0}, {1}, {2}'.format(self.current_gains[i][0],
                                                                self.current_gains[i][1],
                                                                self.current_gains[i][2]))

    def load_current_gains(self):
        self.current_gains = []
        for joint_index in range(len(self._joint_list)):
            gains = rospy.get_param('/march/controller/trajectory/gains/' + self._joint_list[joint_index])
            self.current_gains.append([gains['p'], gains['i'], gains['d']])

    # Method that pulls the PID values from the gains_per_gait_type.yaml config file
    def look_up_table(self, joint_index):
        if rospy.has_param('~gait_types/' + self._gait_type):
            gains = rospy.get_param('~gait_types/' + self._gait_type + '/' + self._joint_list[joint_index])
            return [gains['p'], gains['i'], gains['d']]
        else:
            return [None, None, None]
