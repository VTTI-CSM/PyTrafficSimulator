class Node:
    instances = {}
    def __init__(self, config={}):
        # Set default configuration
        self.set_default_config()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties()

    def set_default_config(self):  
        self.id = 0
        self.coord = (0, 0)


    def init_properties(self):
        Node.instances[self.id] = self