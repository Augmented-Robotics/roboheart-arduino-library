/* This example shows how to create WiFi Access point
 * and calibrate PID for balancing bot.
 * 
 * Created  14/08/2024
 * By Augmented Robotics
 * 
 * Check out https://roboheart.de/en_gb/ for more information about RoboHeart.
 */


#include <RoboHeart.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>


#include <RoboHeart.h>
#include <RoboHeartTimer.h>

RoboHeart heart = RoboHeart(Serial);

// PID controller parameters
int Kp = 0;
int Kd = 0;
int Ki = 0;

#define CONTROL_TICK_PERIOD_US 100.0

#define PID_CONTROL_PRESCALER \
    5  // Control ticks passing before PID motorPower Calculation
#define DC_CONTROL_PRESCALER 15  // Control ticks passing before DC Motor Control
#define STATISTICS_PRESCALER \
    DC_CONTROL_PRESCALER *   \
        4  // Control ticks passing before all the debug printing is performed

// Motor control parameters
float prevAngleDeg = 0;      // Store previos angle
float motorPower = 0;        // Calculated Power to be supplied to the motors
float errorSum = 0;          // For Integral part of the PID controller
float currentAngleDeg = 0;   // Current Angle
float targetAngleDeg = 0.;   // Target Angle for the Balancing bot
float offsetAngleDeg = 0.;   // Stable vertical position angle

// Track the timer activation for each task individually
unsigned long pidControlTick = 0;
unsigned long dcControlTick = 0;
unsigned long statisticsTick = 0;

// Store previous time
unsigned long prevTimeIntervalMS = 0;

// handle -180 after crossing 180 Degrees
float processAngle(float angle) {
    // handle -180 after crossing 180
    if (angle > 180) {
        return angle - 360;
    }
    return angle;
}

// Periodic timer executes the control ticks
void tick() {
    // reduce code to minimal in order to avoid errors
    pidControlTick++;
    dcControlTick++;
    statisticsTick++;
}

// When Button is pressed the current angle is saved
// and later used to indicate stable vertical position
// of the Balancing Bot.
void processPinInterrupt() {
    heart.resetGyro();
    errorSum = 0;
}

// Periodic timer executes the control ticks
PeriodicTimer timer = PeriodicTimer(tick, CONTROL_TICK_PERIOD_US, Serial);

#define ROBOHEART_AP_SSID       "RoboHeart PID Fitting"
#define ROBOHEART_AP_PASSWORD   "Hercules4Makers"

AsyncWebServer server(80);
// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

const char* PARAM_INPUT_1 = "param_Kp";
const char* PARAM_INPUT_2 = "param_Ki";
const char* PARAM_INPUT_3 = "param_Kd";

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>RoboHeart PID Fitting Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <body><h1>RoboHeart PID Fitting</h1>
  <form action="/setKp">
    Kp: <input type="text" name="param_Kp">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/setKi">
    Ki: <input type="text" name="param_Ki">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/setKd">
    Kd: <input type="text" name="param_Kd">
    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";

void setup()
{
    Serial.begin(115200);
    Serial.println("\nCreating AP.");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ROBOHEART_AP_SSID, ROBOHEART_AP_PASSWORD);
    Serial.print("AP Created with IP Gateway ");
    Serial.println(WiFi.softAPIP());

    // Send web page with input fields to client
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });

    // Send a GET request to <ESP_IP>/setKp?param_Kp=<inputMessage>
    server.on("/setKp", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String inputMessage;
        String inputParam;
        // GET Kp value on <ESP_IP>/setKp?param_Kp=<inputMessage>
        if (request->hasParam(PARAM_INPUT_1)) {
            inputMessage = request->getParam(PARAM_INPUT_1)->value();
            inputParam = PARAM_INPUT_1;
            Kp = atoi(inputMessage.c_str());
        }
        else {
            inputMessage = "No message sent";
            inputParam = "none";
        }
        Serial.println(inputMessage);
        request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
                                        + inputParam + ") with value: " + inputMessage +
                                        "<br><a href=\"/\">Return to Home Page</a>");
    });

    // Send a GET request to <ESP_IP>/setKi?param_Ki=<inputMessage>
    server.on("/setKi", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String inputMessage;
        String inputParam;
        // GET Kp value on <ESP_IP>/setKi?param_Ki=<inputMessage>
        if (request->hasParam(PARAM_INPUT_2)) {
            inputMessage = request->getParam(PARAM_INPUT_2)->value();
            Ki = atoi(inputMessage.c_str());
            inputParam = PARAM_INPUT_2;
        }
        else {
            inputMessage = "No message sent";
            inputParam = "none";
        }
        Serial.println(inputMessage);
        request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
                                        + inputParam + ") with value: " + inputMessage +
                                        "<br><a href=\"/\">Return to Home Page</a>");
    });

    // Send a GET request to <ESP_IP>/setKd?param_Kd=<inputMessage>
    server.on("/setKd", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String inputMessage;
        String inputParam;
        // GET Kp value on <ESP_IP>/setKd?param_Kd=<inputMessage>
        if (request->hasParam(PARAM_INPUT_3)) {
            inputMessage = request->getParam(PARAM_INPUT_3)->value();
            inputParam = PARAM_INPUT_3;
            Kd = atoi(inputMessage.c_str());
        }
        else {
            inputMessage = "No message sent";
            inputParam = "none";
        }
        Serial.println(inputMessage);
        request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
                                        + inputParam + ") with value: " + inputMessage +
                                        "<br><a href=\"/\">Return to Home Page</a>");
    });

    server.begin();

    // Initialize RoboHeart with or without request for IMU automatic
    // calibration
    heart.begin();

    //Run IMU calibration and calculation of absolute rotation
    heart.setAutomaticRotation();

    // Configure Button which is used to set the
    // stable vertical position of the Balancing Bot.
    pinMode(BUTTON_ROBOHEART, INPUT);
    attachInterrupt(BUTTON_ROBOHEART, processPinInterrupt, FALLING);

    delay(100); // Resolve false triggering of button during flashing
    offsetAngleDeg = 0;
    targetAngleDeg = 0;

    // Save current time and start timer
    prevTimeIntervalMS = millis();
    timer.start();
}

void loop(){
    // Perform PID control every CONTROL_TICK_PERIOD_US*PID_CONTROL_PRESCALER
    if (pidControlTick >= PID_CONTROL_PRESCALER) {
        unsigned long curTimeIntervalMS = millis();
        pidControlTick = 0;
        currentAngleDeg = processAngle(heart.getRotationX());

        float error = currentAngleDeg - targetAngleDeg;
        errorSum = constrain(errorSum + error, -Kp * 50, Kp * 50);

        float sampleTimeS = .001 * (curTimeIntervalMS - prevTimeIntervalMS);

        // Calculate and sum output for P, I and D values
        motorPower = Kp * (error) + Ki * (errorSum)*sampleTimeS -
                     Kd * (currentAngleDeg - prevAngleDeg) / sampleTimeS;
        prevAngleDeg = currentAngleDeg;
        prevTimeIntervalMS = curTimeIntervalMS;
    }

    // Perform Motor control every CONTROL_TICK_PERIOD_US*DC_CONTROL_PRESCALER
    if (dcControlTick >= DC_CONTROL_PRESCALER) {
        dcControlTick = 0;
        if (motorPower > 0) {
            heart.motorA.reverse(motorPower);
            heart.motorB.reverse(motorPower);
        } else if (motorPower < 0) {
            heart.motorA.forward(-motorPower);
            heart.motorB.forward(-motorPower);
        }
    }

    // Print statistics every CONTROL_TICK_PERIOD_US*STATISTICS_PRESCALER
    if (statisticsTick >= STATISTICS_PRESCALER) {
        statisticsTick = 0;
        Serial.print(" P :");
        Serial.print(motorPower);
        Serial.print(" A :");
        Serial.print(currentAngleDeg);
        Serial.print(" AT :");
        Serial.print(targetAngleDeg);
        Serial.print(" ES :");
        Serial.println(errorSum);
    }

    delay(3);
}