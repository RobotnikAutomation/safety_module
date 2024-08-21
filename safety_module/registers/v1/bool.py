from . import RegisterBase

from time import sleep

class Bool(RegisterBase):
    def __init__(self, name, description, config):
        super().__init__(name, description)
        self._config = config
        self.__address = config['address']
        self.__kind = config['kind']
        self.__trigger = config['trigger'] if 'trigger' in config else False
        self.set_context(False)


    def process(self, data):
        if self.__kind == 'input':
            self.set_context(data[self.__address] == 1)


    def write(self, value, set_value_callback):
        if self.__kind == 'input':
            print(f'Cannot write to input register {self.get_name()}')
            return

        if self.__trigger and value:
            set_value_callback(self.__address, False)
            sleep(0.25)
        set_value_callback(self.__address, value)
        self.set_context(value)
