import abc

class BaseFilter(abc.ABC):
    """
    base class for the filters
    """
    def __init__(self, sensor):
        """

        :param sensor: sensor
        :type sensor:  Sensor.Sensor
        """
        self.sensor = sensor
        self.values = []

    @abc.abstractmethod
    def update(self, value):
        """
        update the values with the filter
        :param value: new values
        :return: updated value
        """
        return value