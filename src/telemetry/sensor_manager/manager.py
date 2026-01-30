from typing import Optional
import board
import busio

import RPi.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

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

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(self.i2c)

    def read_data_digital(self, sensor_name: str) -> int:
        if sensor_name not in self.sensors_config:
            raise ValueError(f"Sensor '{sensor_name}' not found in configuration")
        
        self.pin = self.sensors_config[sensor_name]
        data = GPIO.input(self.pin)
            
        return data
    
    def read_data_analog(self, channel: int, gain: Optional[int] = None) -> tuple:
        if channel < 0 or channel > 3:
            raise ValueError("Channel must be between 0 and 3 for ADS1115")
        if gain is not None and gain not in [1, 2, 4, 8, 16]:
            raise ValueError("Gain must be one of the following values: 1, 2, 4, 8, 16")
        
        if gain is not None:
            self.ads.gain = gain

        analog_channel = AnalogIn(self.ads, channel)

        return analog_channel.value, analog_channel.voltage

    def cleanup(self):
        GPIO.cleanup()
