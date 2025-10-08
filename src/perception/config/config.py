"""
This module provides a class for loading and accessing configuration settings from a YAML file.
"""

import yaml

"""
Configuration Class to load and access configurations for perception tasks.
"""
class Config:
    def __init__(self, config_file):
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)
        
        if self.config is None:
            raise ValueError(f"Configuration file {config_file} is empty or invalid.")
        
        if not isinstance(self.config, dict):
            raise ValueError(f"Configuration file {config_file} does not contain a valid dictionary structure.")

    def get(self, key, default=None):
        return self.config.get(key, default)
