import os

import roslib
import rospy
from sound_play.libsoundplay import SoundClient


class Sounds:
    def __init__(self):
        self._sound_client = SoundClient()

        sounds_dir = os.path.join(roslib.packages.get_pkg_dir('march_state_machine'), 'sounds')
        self._sounds = {'start': self._sound_client.waveSound(sounds_dir + '/start.wav')}

    def play(self, sound):
        """
        Plays the given sound if it was loaded.

        :type sound: str
        :param sound: Name of the sound to play
        """
        if sound in self._sounds:
            rospy.loginfo('playing sound {0}'.format(sound))
            self._sounds[sound].play()
        else:
            rospy.logwarn('Sound {0} is not available'.format(sound))
