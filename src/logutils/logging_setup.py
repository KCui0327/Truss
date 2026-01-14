import logging
import yaml
from pathlib import Path

_DEFAULT_CONFIG = Path(__file__).with_name("config.yaml")

def init():
    """
    Initialize the logging configuration from a YAML file.
    """

    config_path = _DEFAULT_CONFIG
    if not config_path.exists():
        raise FileNotFoundError(f"Logging configuration file not found: {config_path}")
    
    with config_path.open() as f:
        config = yaml.safe_load(f)
    
    logging.config.dictConfig(config)

def get_logger(name: str):
    """
    Get a logger with the specified name.
    """

    return logging.getLogger(name)
