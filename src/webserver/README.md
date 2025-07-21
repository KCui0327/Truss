This is a webserver that Truss sends data over to and the Stolon (mobile app) listens for updates from the web server.
The web server will push updates to Stolon for any changes in temperature, pH, and humidity data detected by sensors.

There are endpoints for each sensor and they are `/<sensor_data_category>`.