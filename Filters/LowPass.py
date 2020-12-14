from . import BaseFilter

class LowPass(BaseFilter.BaseFilter):
    """
    implentation of a low pass filter
    """
    def __init__(self,  alpha):
        self._alpha = alpha
        self.y = None

    def update(self, value):
        if self.y is None:
            self.y = value
        self.y = self.y + self._alpha * (value - self.y)

        return self.y