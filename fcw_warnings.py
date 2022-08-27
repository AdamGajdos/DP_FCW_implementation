from abc import ABC, abstractmethod
# from enum import Enum
#
#
# class DangerLevel(Enum):
#     SAFE = 0
#     LEVEL1 = 1
#     LEVEL2 = 2
#     LEVEL3 = 3


class FCWWarning(ABC):

    @abstractmethod
    def __init__(self, **kwargs):
        pass

    @abstractmethod
    def resolve_warning(self):
        pass
