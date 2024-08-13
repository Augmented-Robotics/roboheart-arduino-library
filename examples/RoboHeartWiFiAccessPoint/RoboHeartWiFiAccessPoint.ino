/* This example shows how to create WiFi Access point
 * and control built-in LED.
 * 
 * Created  30/01/2022
 * By Augmented Robotics
 * 
 * Check out https://roboheart.de/en_gb/ for more information about RoboHeart.
 */


#include <RoboHeart.h>
#include <WiFi.h>

#define ROBOHEART_AP_SSID       "RoboHeart WiFi"
#define ROBOHEART_AP_PASSWORD   "Hercules4Makers"

WiFiServer server(80);
// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup()
{
    Serial.begin(115200);
    Serial.println("\nCreating AP.");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ROBOHEART_AP_SSID, ROBOHEART_AP_PASSWORD);
    Serial.print("AP Created with IP Gateway ");
    Serial.println(WiFi.softAPIP());

    // Configure the LED pin
    pinMode(LED_ROBOHEART, OUTPUT);
    server.begin();
}

void loop(){
    WiFiClient client = server.available();   // Listen for incoming clients
    String header;
    if (client) {  
        currentTime = millis();
        previousTime = currentTime;
        Serial.println("New Client.");          // print a message out in the serial port
        String currentLine = "";                // make a String to hold incoming data from the client
        while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
        currentTime = millis();
        if (client.available()) {             // if there's bytes to read from the client,
            char c = client.read();             // read a byte, then
            Serial.write(c);                    // print it out the serial monitor
            header += c;
            if (c == '\n') {                    // if the byte is a newline character
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
                if (currentLine.length() == 0) {
                    // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
                    // and a content-type so the client knows what's coming, then a blank line:
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-type:text/html");
                    client.println("Connection: close");
                    client.println();
                    
                    // turns the GPIOs on and off
                    if (header.indexOf("GET /LED_ROBOHEART/on") >= 0) {
                        Serial.println("LED_ROBOHEART on");
                        digitalWrite(LED_ROBOHEART, HIGH);
                    } else if (header.indexOf("GET /LED_ROBOHEART/off") >= 0) {
                        Serial.println("LED_ROBOHEART off");
                        digitalWrite(LED_ROBOHEART, LOW);
                    }
                    
                    // Display the HTML web page
                    client.println("<!DOCTYPE html><html>");
                    client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
                    client.println("<link rel=\"icon\" href=\"data:,\">");
                    // CSS to style the on/off buttons 
                    // Feel free to change the background-color and font-size attributes to fit your preferences
                    client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
                    client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
                    client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
                    client.println(".button2 {background-color: #555555;}</style></head>");
                    
                    // Web Page Heading
                    client.println("<body><h1>RoboHeart Access Point Example</h1>");
                    
                    // Display current state, and ON/OFF buttons for LED_ROBOHEART  
                    client.println("<p>LED_ROBOHEART - State " + String((digitalRead(LED_ROBOHEART) == HIGH) ? "on" : "off")  + "</p>");

                    if ((digitalRead(LED_ROBOHEART))==LOW) {
                        client.println("<p><a href=\"/LED_ROBOHEART/on\"><button class=\"button\">LED ON</button></a></p>");
                    } else {
                        client.println("<p><a href=\"/LED_ROBOHEART/off\"><button class=\"button button2\">LED OFF</button></a></p>");
                    } 
                    
                    client.println("</body></html>");
                    
                    // The HTTP response ends with another blank line
                    client.println();
                    // Break out of the while loop
                    break;
                } else { // if you got a newline, then clear currentLine
                    currentLine = "";
                }
            } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
            }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
    }
}