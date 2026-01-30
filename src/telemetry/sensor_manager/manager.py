import RPi.GPIO as GPIO

# Manage sensor data collection via GPIO pins
class SensorManager:
    def __init__(self, sensors_config: dict):
        if len(sensors_config) == 0:
            raise ValueError("sensors_config cannot be empty")
        if not isinstance(sensors_config, dict):
            raise TypeError("sensors_config must be a dictionary")
        
        self.sensors_config = sensors_config
        GPIO.setmode(GPIO.BCM)
        pins = sensors_config.values()
        for pin in pins:
            GPIO.setup(pin, GPIO.IN)

    def read_data(self, sensor_name: str):
        if sensor_name not in self.sensors_config:
            raise ValueError(f"Sensor '{sensor_name}' not found in configuration")
        
        self.pin = self.sensors_config[sensor_name]
        data = GPIO.input(self.pin)
        return data

    def cleanup(self):
        GPIO.cleanup()
