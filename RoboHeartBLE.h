/*!
 *  @file RoboHeart.h
 *
 *
 *	MIT license (see license.txt)
 */
#ifndef _ROBOHEARTBLE_H
#define _ROBOHEARTBLE_H

#include <Arduino.h>

#include <BLECharacteristic.h>
#include <BLEAdvertising.h>
#include <BLEServer.h>

#define RH_APP_SERVICE_UUID         "000000ff-0000-1000-8000-00805f9b34fb"
#define RH_APP_CHARACTERISTIC_UUID1 "0000ff01-0000-1000-8000-00805f9b34fb"
#define RH_APP_CHARACTERISTIC_UUID2 "0000ff02-0000-1000-8000-00805f9b34fb"
#define RH_APP_CHARACTERISTIC_UUID3 "0000ff03-0000-1000-8000-00805f9b34fb"

typedef struct { 
    std::string service; 
    std::string char1; 
    std::string char2; 
    std::string char3; 
} BLE_UUID_Config_t;

class InterfaceBLE
{
    public:
        InterfaceBLE(Stream& debug);
        InterfaceBLE();

        static void serverOnConnectInvCallback();
        static void serverOnDisconnectInvCallback();
        static void charOnWriteInvCallback(std::string uuid, std::string value);

        void configure(uint8_t* package, uint8_t package_size, BLE_UUID_Config_t* config_uuids = NULL);

        void setCharacteristicsCallbacks(void (*char1write) (std::string), void (*char2write) (std::string), void (*char3write) (std::string));
        void setServerCallbacks(void (*onConnect) (void), void (*onDisconnect) (void));
        
        bool StartServiceAdvertising();
        bool StopServiceAdvertising();
        bool sendNotifyChar2(uint8_t* package);

    private:
        static BLE_UUID_Config_t* ble_uuids;

        static void (*char1writeCallback) (std::string);
        static void (*char2writeCallback) (std::string);
        static void (*char3writeCallback) (std::string);
        static void (*serverOnConnectCallback) (void);
        static void (*serverOnDisconnectCallback) (void);
        
        BLEServer* pServer;
        BLEAdvertising *pAdvertising;
        BLECharacteristic *pCharacteristic1;
        BLECharacteristic *pCharacteristic2;
        BLECharacteristic *pCharacteristic3;

        Stream* _debug;  
        hw_timer_t * timer;   
        uint8_t packageCharSize;
        bool configured;
};


#endif // _ROBOHEARTBLE_H