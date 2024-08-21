from . import RegisterBase

class Trigger(RegisterBase):
    def __init__(self, name, description, config):
        super().__init__(name, description)
        self._config = config
        self.__address = config['address']
        self.set_context(False)

    def write(self, value, set_value_callback):
        print(f'Writing {self.get_name()} to {self.__address} with value {value}')
        if value:
            set_value_callback(self.__address, False)
        set_value_callback(self.__address, value)
        self.set_context(value)
