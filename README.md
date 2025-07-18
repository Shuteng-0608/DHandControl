
# DexHandControl Library

This DexHandControl library is developed based on a UDP communication framework. Below are the configuration and usage instructions:

## Configuration

1. Navigate to `DHandControl/BusServoDriverHAT` and upload this Arduino program to your ESP32 control board. Pay special attention to the following configurations in the `UDP.h` file:

```cpp
// WIFI_STA settings
const char* STA_SSID = "Shuteng";
const char* STA_PWD  = "12345678";

WiFiUDP Udp;
const int udpPort = 12345; // Port number for UDP communication
IPAddress local_ip(192, 168, 4, 5); // Static local IP   
IPAddress gateway(192, 168, 4, 1); // AP gateway    
IPAddress subnet(255, 255, 255, 0);
```

### Configuration Details

- `STA_SSID` and `STA_PWD`: These represent the WiFi network name and password that the ESP32 will connect to.
- `gateway`: This is the gateway address of the local network access point (AP).
- `local_ip`: Defines the static IP address of the ESP32 within this local network.
- `udpPort`: Specifies the port number used for establishing UDP communication.

