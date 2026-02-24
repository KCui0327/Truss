import logging
import logging.config
import logging_loki
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
    Get loki logger for module. Used for observability in Grafana Loki.
    """
    loki_logger = logging.getLogger(name)
    loki_handler = logging_loki.LokiHandler(
        url="http://localhost:3100/loki/api/v1/push",
        version="1",
        tags={"service_name": name},
    )
    loki_logger.setLevel(logging.DEBUG)
    loki_logger.addHandler(loki_handler)

    return loki_logger
