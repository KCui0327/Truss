from sensor_manager.manager import SensorManager

class Telemetry:
    def __init__(self, sensors_config: dict):
        if len(sensors_config) == 0:
            raise ValueError("sensors_config cannot be empty")
        if not isinstance(sensors_config, dict):
            raise TypeError("sensors_config must be a dictionary")
        
        self.sensor_manager = SensorManager(sensors_config)

    # TODO: Add methods to collect and process different sensor data
    