
const char* ssid = "Casadevelez";
const char* password = "Nosoup4you!";
const char* hostname = "esp32-server";

AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// Json Variable to Hold Sensor Readings
JsonDocument readings;