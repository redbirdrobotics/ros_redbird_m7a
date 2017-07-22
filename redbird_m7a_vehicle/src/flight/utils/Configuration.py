#!/usr/bin/python

"""Configuration.py: Loads configuration values from a .yaml file."""

import yaml

__author__ = "Alex Bennett"


class Configuration(object):
    def __init__(self, file_name):
        # Open file
        self._config_file = open(file_name, 'r')

        # Parse YAML
        self._config_yaml = yaml.load(self._config_file)

        # Close file
        self._config_file.close()

    def get_yaml(self):
        return self._config_yaml
