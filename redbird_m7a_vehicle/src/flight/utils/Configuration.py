import yaml

# TODO: Add header docstring

class Configuration:
    def __init__(self, file_name):
        # Open file
        self._config_file = open(file_name, 'r')

        # Parse YAML
        self._config_yaml = yaml.load(self._config_file)

        # Close file
        self._config_file.close()

    def get_yaml(self):
        return self._config_yaml


