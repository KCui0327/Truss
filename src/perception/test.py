import logging
import logging_loki
import time

handler = logging_loki.LokiHandler(
    url="http://localhost:3100/loki/api/v1/push",
    version="1", 
)

logger = logging.getLogger("my-app")
logger.setLevel(logging.INFO) # Ensure the logger level allows INFO
logger.addHandler(handler)

# Send the log
logger.info("Testing Loki connection")

print("Log sent, waiting for flush...")
time.sleep(3) 
print("Done.")