from importlib import import_module

def snake_to_camel(snake_str):
    return ''.join(word.capitalize() for word in snake_str.split('_'))

def get_register(version, config):
    # Extract the register name, description and type
    register_name = config.pop('name')
    register_description = config.pop('description')
    register_type = version + '.' + config.pop('type')
    register_module = register_type.split('.')

    # Load the register class
    global_module_name = 'safety_module.registers.' + register_type
    register = getattr(import_module(global_module_name), snake_to_camel(register_module[-1]))
    return register(register_name, register_description, config)


def load_registers(register_config) -> list:
    ret = []
    for version, register_list in register_config.items():
        for r in register_list:
            ret.append(get_register(version, r))

    return ret
