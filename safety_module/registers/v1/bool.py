from . import RegisterBase

from time import sleep


class Bool(RegisterBase):
    def __init__(self, name, description, config):
        super().__init__(name, description, config["kind"])
        self._config = config
        self.__address = config["address"]
        self.__trigger = config["trigger"] if "trigger" in config else False
        self.set_context(
            {
                "value": None,
                "description": "Unititialized",
            }
        )

    def get_type(self):
        return "Bool.v1"

    def process(self, data):
        if self.kind() == "output":
            return

        # Get the value from the data
        value_data = data[self.__address] == 1
        value_str = str(value_data).lower()
        value_description = self.get_value_description(value_str)
        self.set_context(
            {
                "value": value_data,
                "description": value_description,
            }
        )

    def write(self, value, set_value_callback):
        if self.kind() == "input":
            print(f"Cannot write to input register {self.get_name()}")
            return

        if self.__trigger and value:
            set_value_callback(self.__address, False)
            sleep(0.25)
        set_value_callback(self.__address, value)
        self.set_context(value)
