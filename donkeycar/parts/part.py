from abc import ABC, abstractmethod
from typing import List


class Part(ABC):

    @abstractmethod
    def run(self, **kw):
        pass

    def shutdown(self):
        pass

    @abstractmethod
    def get_inputs_keys(self) -> List[str]:
        pass

    @abstractmethod
    def get_outputs_keys(self) -> List[str]:
        pass


class ThreadedPart(Part):
    def run(self, **kw):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def run_threaded(self, **kw):
        pass
