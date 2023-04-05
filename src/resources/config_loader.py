import os

class ConfigLoader:
    def __init__(self):
        self.config = {}

    def load(self, env_vars):
        for env_var in env_vars:
            print(os.environ.get(env_var))
            self.config[env_var] = os.environ.get(env_var)

    def __getitem__(self, key):
        return self.config[key]

    def __setitem__(self, key, value):
        self.config[key] = value

    def __contains__(self, key):
        return key in self.config
