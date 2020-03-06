from dynamic_reconfigure.client import Client
import rospy

from march_shared_resources.msg import GaitActionGoal

from .one_step_linear_interpolation import interpolate


class DynamicPIDReconfigurer:
    def __init__(self, joint_list):
        self._gait_type = None
        self._joint_list = joint_list
        self.interpolation_done = True
        self._last_update_times = []
        self._clients = [Client('/march/controller/trajectory/gains/' + joint, timeout=30) for joint in
                         self._joint_list]
        self.current_gains = []
        rospy.Subscriber('/march/gait/schedule/goal', GaitActionGoal, callback=self.gait_selection_callback)
        self._linearize = rospy.get_param('~linearize_gain_scheduling')
        self._gradient = rospy.get_param('~linear_slope')

    def gait_selection_callback(self, data):
        new_gait_type = data.goal.current_subgait.gait_type
        if new_gait_type is None or new_gait_type == '':
            new_gait_type = 'walk_like'
            rospy.logwarn('The gait has no gait type, default is set to walk_like')

        if self._gait_type != new_gait_type or not self.interpolation_done:
            self._gait_type = new_gait_type
            self.load_current_gains()
            self.interpolation_done = False

            needed_gains = [self.look_up_table(i) for i in range(len(self._joint_list))]
            rate = rospy.Rate(10)

            rospy.loginfo('Beginning PID interpolation for gait type: {0}'.format(self._gait_type))
            begin_time = rospy.get_time()
            self._last_update_times = len(self._joint_list) * [begin_time]
            while not self.interpolation_done:
                self.client_update(needed_gains)
                rate.sleep()
            rospy.loginfo('PID interpolation finished in {0}s'.format(rospy.get_time() - begin_time))

    def client_update(self, needed_gains):
        self.interpolation_done = True

        for i in range(len(self._joint_list)):
            if self._linearize:
                current_time = rospy.get_time()
                time_interval = current_time - self._last_update_times[i]

                self.current_gains[i] = interpolate(self.current_gains[i], needed_gains[i], self._gradient,
                                                    time_interval)
                if self.current_gains[i] != needed_gains[i]:
                    self.interpolation_done = False

                self._last_update_times[i] = rospy.get_time()
            else:
                self.current_gains[i] = needed_gains[i]

            self._clients[i].update_configuration({'p': self.current_gains[i][0],
                                                   'i': self.current_gains[i][1],
                                                   'd': self.current_gains[i][2]})

    def load_current_gains(self):
        self.current_gains = []
        for joint_name in self._joint_list:
            gains = rospy.get_param('/march/controller/trajectory/gains/' + joint_name)
            self.current_gains.append([gains['p'], gains['i'], gains['d']])

    # Method that pulls the PID values from the gains_per_gait_type.yaml config file
    def look_up_table(self, joint_index):
        gains = rospy.get_param('~gait_types/' + self._gait_type + '/' + self._joint_list[joint_index],
                                {'p': None, 'i': None, 'd': None})
        return [gains['p'], gains['i'], gains['d']]
