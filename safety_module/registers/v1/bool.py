from . import RegisterBase

class Bool(RegisterBase):
    def __init__(self, name, description, config):
        super().__init__(name, description)
        self._config = config
        self.__address = config['address']
        self.__kind = config['kind']
        self.set_context(False)


    def process(self, data):
        if self.__kind == 'input':
            self.set_context(data[self.__address] == 1)


    def write(self, value, set_value_callback):
        if self.__kind == 'input':
            print(f'Cannot write to input register {self.get_name()}')
        else:
            print(f'Writing {self.get_name()} to {self.__address} with value {value}')
            set_value_callback(self.__address, value)
            self.set_context(value)
