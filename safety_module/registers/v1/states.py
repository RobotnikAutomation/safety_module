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
        if self.__kind == 'output':
            return

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

    def write(self, value, set_value_callback):
        if self.__kind == 'input':
            print(f'Cannot write to input register {self.get_name()}')
            return

        print(f'Getting value from {self.get_name()} with value {value}')
        # Get value from state name
        state = None
        if isinstance(value, str):
            for s in self.__states:
                if s['name'] == value:
                    state = s
                    break
            else:
                print(f'Unknown state {value}')
                return

        print(f'Writing {self.get_name()} with value {s["value"]}')
        for i in range(self.__num_bits):
            set_value_callback(self.__base_address + i, bool((s["value"] >> i) & 1))

        self.set_context(state)
