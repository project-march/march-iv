import os

import roslib
import rospy
from sound_play.libsoundplay import SoundClient


class Sounds:
    def __init__(self):
        self._sound_client = SoundClient()

        self._sounds = {}
        self._sounds_dir = os.path.join(roslib.packages.get_pkg_dir('march_state_machine'), 'sounds')

    def add_sound(self, sound):
        """
        Adds a sound if it was not already added.

        :type sound: str
        """
        if sound in self._sounds:
            rospy.logwarn('Sound {0} already in sounds'.format(sound))
        else:
            self._sounds[sound] = self._sound_client.waveSound(os.path.join(self._sounds_dir, '{0}.wav'.format(sound)))

    def play(self, sound):
        """
        Plays the given sound if it was loaded.

        :type sound: str
        :param sound: Name of the sound to play
        """
        if sound in self._sounds:
            rospy.logdebug('playing sound {0}'.format(sound))
            self._sounds[sound].play()
        else:
            rospy.logwarn('Sound {0} is not available'.format(sound))
