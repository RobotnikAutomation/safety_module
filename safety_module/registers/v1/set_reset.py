from . import RegisterBase

class SetReset(RegisterBase):
    def __init__(self, name, description, config):
        super().__init__(name, description)
        self._config = config
        self.__set_address = config['set_address']
        self.__reset_address = config['reset_address']
        self.set_context(False)

    def write(self, value, set_value_callback):
        if value:
            set_value_callback(self.__reset_address, 0)
            set_value_callback(self.__set_address, 1)
        else:
            set_value_callback(self.__set_address, 0)
            set_value_callback(self.__reset_address, 1)
        self.set_context(value)
