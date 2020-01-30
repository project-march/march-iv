class GaitError(Exception):
    def __init__(self, msg=None):
        """Base class for exceptions in gait modules.

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = 'An error occurred with a gait module.'
        super(GaitError, self).__init__(msg)


class GaitNameNotFound(GaitError):
    def __init__(self, gait_name, msg=None):
        """Class to raise an error when given gait name does not exists .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = 'Could not find gait name: {gn} in map.'.format(gn=gait_name)

        super(GaitNameNotFound, self).__init__(msg)


class SubgaitNameNotFound(GaitError):
    def __init__(self, subgait_name=None, msg=None):
        """Class to raise an error when given subgait name does not exists .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = 'Could not find subgait name: {gn} in map.'.format(gn=subgait_name)

        super(SubgaitNameNotFound, self).__init__(msg)


class NonValidGaitContent(GaitError):
    def __init__(self, gait_name=None, msg=None):
        """Class to raise an error when given gait has incorrect content .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = 'The given gait: {gn} has incorrect information'.format(gn=gait_name)

        super(NonValidGaitContent, self).__init__(msg)


class TransitionError(Exception):
    def __init__(self, msg=None):
        """Class to raise an error when transition between two subgaits has an error .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = 'Subgaits can not transition'

        super(TransitionError, self).__init__(msg)
