
class FileNotFoundError(Exception):
    def __init__(self, file_path, msg=None):
        """Class to raise an error when a file cannot be found.

        :param file_path:
            The file path which is not found by os.path.isfile()
        """
        if msg is None:
            msg = 'File path: {fp} could not be found.'.format(fp=file_path)

        super(FileNotFoundError, self).__init__(msg)


class PackageNotFoundError(Exception):
    def __init__(self, package_name, msg=None):
        """Class to raise an error when a ros package cannot be found.

        :param package_name:
            The package name which is not found by rospkg.RosPack().get_path()
        """
        if msg is None:
            msg = 'Package: {fp} could not be found.'.format(fp=package_name)

        super(PackageNotFoundError, self).__init__(msg)


class MsgTypeError(Exception):
    def __init__(self, msg=None):
        """Class to raise an error when an non msg type is added to a message."""
        if msg is None:
            msg = 'A non msg type (defined in shared resources) was added to a ROS-message'

        super(MsgTypeError, self).__init__(msg)
