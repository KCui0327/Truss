import requests
import time
import random

# Server configuration
URL_TEMP = "http://192.168.2.83:12345/temperature"
URL_HUMIDITY = "http://192.168.2.83:12345/humidity"

def send_payloads(iterations=5, delay=5):
    print(f"Starting transmission of {iterations} payloads...\n")
    
    for i in range(1, iterations + 1):
        dataTemp = {
            "temperature": round(random.uniform(14.0, 19.0), 2)
        }

        dataHumidity = {
            "humidity": round(random.uniform(62.0, 78.0), 2)
        }
        
        try:
            # Sending the POST request as JSON
            requests.post(URL_TEMP, json=dataTemp, timeout=5)
            requests.post(URL_HUMIDITY, json=dataHumidity, timeout=5)
            
        except requests.exceptions.ConnectionError:
            print(f"[{i}] Error: Could not connect to the server. Is it running?")
        except Exception as e:
            print(f"[{i}] An error occurred: {e}")
        
        # Wait for 2 seconds before the next iteration, unless it's the last one
        if i < iterations:
            time.sleep(delay)

    print("\nTransmission complete.")

if __name__ == "__main__":
    send_payloads()