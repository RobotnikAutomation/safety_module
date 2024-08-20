from . import RegisterBase

class States(RegisterBase):
    def __init__(self, name, description, config):
        super().__init__(name, description)
        self.__base_address = config['base_address']
        self.__num_bits = config['num_bits']
        self.__bit_order = config['bit_order'] if 'bit_order' in config else 'msbf'
        self.__states = config['states']
        self.__kind = config['kind']


    def process(self, data):
        value = 0
        for i in range(self.__num_bits):
            if self.__bit_order == 'lsbf':
                value += data[self.__base_address + i] * (2 ** i)
            else:
                value += data[self.__base_address + i] * (2 ** (self.__num_bits - 1 - i))

        curr_state = {}
        for s in self.__states:
            if s['value'] == value:
                curr_state = s
                break

        if curr_state:
            self.set_context(curr_state)
        else:
            self.set_context({'name': 'Unknown', 'description': 'Unknown state', 'value': value})
