"""
logging: A centralized logging module for the application.
"""

__version__ = "1.0.0"

from .logging_setup import init, get_logger

__all__ = ["init", "get_logger"]