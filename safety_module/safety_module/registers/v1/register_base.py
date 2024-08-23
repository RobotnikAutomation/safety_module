class RegisterBase:
    def __init__(self, name, description, kind):
        self.__name = name
        self.__description = description
        self.__context = None
        self.__kind = kind

    def process(self, data):
        pass

    def write(self, value, set_value_callback):
        pass

    def get_name(self):
        return self.__name

    def get_description(self):
        return self.__description

    def get_context(self):
        return self.__context

    def set_context(self, context):
        self.__context = context

    def get_value_description(self, value: str) -> str:
        for description_value in self.get_description()["values"]:
            for key, v in description_value.items():
                if key == value:
                    return v
        return "Unknown"

    def __str__(self):
        if self.__context is None:
            return f"{self.__name}: None"

        return f'{self.__name}: {self.get_context()["value"]}'

    def get_type(self):
        return type(self).__name__

    def kind(self):
        return self.__kind
