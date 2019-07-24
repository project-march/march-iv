from SoftwareCheck import SoftwareCheck


class DefaultCheck(SoftwareCheck):

    def __init__(self):
        SoftwareCheck.__init__(self, "Default", "Just an example software check, is always true", 1, False)

    def perform(self):
        self.done = True
        self.passed = True
