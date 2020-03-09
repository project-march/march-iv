from enum import Enum


class Color(Enum):
    Debug = '#009100'
    Info = '#000000'
    Warning = '#b27300'  # noqa A003
    Error = '#FF0000'
    Fatal = '#FF0000'
    Check_Unknown = '#b27300'
    Check_Failed = '#FF0000'
    Check_Passed = '#009100'
