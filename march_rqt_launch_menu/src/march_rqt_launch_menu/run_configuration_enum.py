from enum import Enum


class run_configuration_enum(Enum):
    exoskeleton = "exoskeleton"
    simulation = "simulation"

    @classmethod
    def has_value(cls, value):
        return any(value == item.value for item in cls)
