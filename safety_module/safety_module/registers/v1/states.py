from . import RegisterBase


class States(RegisterBase):
    def __init__(self, name, description, config):
        super().__init__(name, description, config["kind"])
        self.__base_address = config["base_address"]
        self.__num_bits = config["num_bits"]
        self.__bit_order = config["bit_order"] if "bit_order" in config else "msbf"
        self.__states = config["states"]
        self.set_context(
            {
                "value": None,
                "description": "Unititialized",
                "raw_value": None,
            }
        )

    def get_type(self):
        return "States.v1"

    def process(self, data):
        if self.kind() == "output":
            return

        value = 0
        for i in range(self.__num_bits):
            if self.__bit_order == "lsbf":
                value += data[self.__base_address + i] * (2**i)
            else:
                value += data[self.__base_address + i] * (
                    2 ** (self.__num_bits - 1 - i)
                )

        raw_value = value
        curr_state = {}
        for s in self.__states:
            if s["value"] == value:
                curr_state = s
                break

        if curr_state:
            self.set_context(
                {
                    "value": curr_state["name"],
                    "description": self.get_value_description(curr_state["name"]),
                    "raw_value": raw_value,
                }
            )
        else:
            self.set_context(
                {
                    "value": f"unknown_{value}",
                    "description": "Unknown state",
                    "raw_value": raw_value,
                }
            )

    def write(self, value, set_value_callback):
        if self.kind() == "input":
            print(f"Cannot write to input register {self.get_name()}")
            return

        # Get value from state name
        state = None
        if isinstance(value, str):
            for s in self.__states:
                if s["name"] == value:
                    state = s
                    break
            else:
                print(f"Unknown state {value}")
                return

        addressess = [self.__base_address + i for i in range(self.__num_bits)]
        values = [bool((state["value"] >> i) & 1) for i in range(self.__num_bits)]
        set_value_callback(addressess, values)

        self.set_context(state)
