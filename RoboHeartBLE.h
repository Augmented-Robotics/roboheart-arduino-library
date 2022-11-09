/*!
 *  @file RoboHeartBLE.h
 *
 * 	Arduino library for the RoboHeart.
 *
 */
#ifndef RoboHeartBLE_h
#define RoboHeartBLE_h

#include <Arduino.h>
#include <BLEAdvertising.h>
#include <BLECharacteristic.h>
#include <BLEServer.h>

#define RH_APP_SERVICE_UUID "000000ff-0000-1000-8000-00805f9b34fb"
#define RH_APP_CHARACTERISTIC_UUID1 "0000ff01-0000-1000-8000-00805f9b34fb"
#define RH_APP_CHARACTERISTIC_UUID2 "0000ff02-0000-1000-8000-00805f9b34fb"
#define RH_APP_CHARACTERISTIC_UUID3 "0000ff03-0000-1000-8000-00805f9b34fb"

typedef struct {
    std::string service;
    std::string char1;
    std::string char2;
    std::string char3;
} uuidConfigType;

class InterfaceBLE {
   public:
    InterfaceBLE(Stream& debug);
    InterfaceBLE();

    static void serverOnConnectInvCallback();
    static void serverOnDisconnectInvCallback();
    static void charOnWriteInvCallback(std::string uuid, std::string value);

    void begin(uint8_t* package = NULL, uint8_t packageSize = 0,
               uuidConfigType* uuidsConfig = NULL);

    void setCharacteristicsCallbacks(void (*char1Write)(std::string),
                                     void (*char2Write)(std::string),
                                     void (*char3Write)(std::string));
    void setServerCallbacks(void (*onConnect)(void),
                            void (*onDisconnect)(void));

    bool startServiceAdvertising();
    bool stopServiceAdvertising();
    bool sendNotifyChar2(uint8_t* package);
    bool sendNotifyChar3(uint8_t* package);

   private:
    static uuidConfigType* _uuidsBLE;

    static void (*char1WriteCallback)(std::string);
    static void (*char2WriteCallback)(std::string);
    static void (*char3WriteCallback)(std::string);
    static void (*serverOnConnectCallback)(void);
    static void (*serverOnDisconnectCallback)(void);

    BLEServer* _Server;
    BLEAdvertising* _Advertising;
    BLECharacteristic* _characteristic1;
    BLECharacteristic* _characteristic2;
    BLECharacteristic* _characteristic3;

    Stream* _debug = NULL;
    uint8_t _packageCharSize;
    bool _configured = false;
};

#endif  // RoboHeartBLE_h