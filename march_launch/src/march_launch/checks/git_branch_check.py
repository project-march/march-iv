import rospy

from march_launch.Color import Color

from .launch_check import LaunchCheck


class GitBranchCheck(LaunchCheck):

    def __init__(self):
        LaunchCheck.__init__(self,
                             'GitBranch',
                             'Please confirm the branches below',
                             'march_launch',
                             'git_branch.launch',
                             10,
                             True)

    def perform(self):
        self.launch()

        rospy.sleep(4)
        self.stop_launch_process()

        branch_tuples = self.get_key_from_parameter_server('/checks/git_branch')

        for branch_tuple in branch_tuples:
            repository_name, branch_name = branch_tuple
            print_string = repository_name + ': ' + branch_name
            if branch_name == 'develop':
                self.log(print_string, Color.Debug)
            elif 'training/' in branch_name:
                self.log(print_string, Color.Debug)
            else:
                self.log(print_string, Color.Warning)

        self.done = True
        self.passed = True
