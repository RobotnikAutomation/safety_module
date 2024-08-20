class RegisterBase:
    def __init__(self, name, description):
        self.__name = name
        self.__description = description
        self.__context = None

    def process(self, data):
        raise NotImplementedError
    
    def get_name(self):
        return self.__name
    
    def get_description(self):
        return self.__description
    
    def get_context(self):
        return self.__context
    
    def set_context(self, context):
        self.__context = context

    def write(self, value, set_value_callback):
        pass
