from . import RegisterBase


class Trigger(RegisterBase):
    def __init__(self, name, description, config):
        super().__init__(name, description, config["kind"])
        self._config = config
        self.__address = config["address"]
        self.set_context(
            {
                "value": None,
                "description": "Unititialized",
            }
        )

    def get_type(self):
        return "Trigger.v1"

    def write(self, value, set_value_callback):
        if value:
            set_value_callback(self.__address, False)
        set_value_callback(self.__address, value)
        self.set_context(
            {
                "value": value,
                "description": "Triggered" if value else "Untriggered",
            }
        )
