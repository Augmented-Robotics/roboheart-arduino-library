/*!
 *  @file RoboHeartBLE.cpp
 *
 * 	Arduino library for the RoboHeart.
 *
 */

#include "RoboHeartBLE.h"

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEUtils.h>

#define DEBUG_BLE(x)                       \
    {                                      \
        if (_debug != NULL) {              \
            _debug->print("[BLE_DEBUG] "); \
            _debug->print(x);              \
        }                                  \
    }
#define DEBUG_LN_BLE(x)                    \
    {                                      \
        if (_debug != NULL) {              \
            _debug->print("[BLE_DEBUG] "); \
            _debug->println(x);            \
        }                                  \
    }
#define DEBUG(x)              \
    {                         \
        if (_debug != NULL) { \
            _debug->print(x); \
        }                     \
    }
#define DEBUG_LN(x)             \
    {                           \
        if (_debug != NULL) {   \
            _debug->println(x); \
        }                       \
    }

static BLE_UUID_Config_t uuids = {
    RH_APP_SERVICE_UUID, RH_APP_CHARACTERISTIC_UUID1,
    RH_APP_CHARACTERISTIC_UUID2, RH_APP_CHARACTERISTIC_UUID3};

BLE_UUID_Config_t* InterfaceBLE::ble_uuids = &uuids;

void (*InterfaceBLE::char1writeCallback)(std::string) = NULL;
void (*InterfaceBLE::char2writeCallback)(std::string) = NULL;
void (*InterfaceBLE::char3writeCallback)(std::string) = NULL;
void (*InterfaceBLE::serverOnConnectCallback)(void) = NULL;
void (*InterfaceBLE::serverOnDisconnectCallback)(void) = NULL;

InterfaceBLE::InterfaceBLE(Stream& debug) : _debug(&debug), configured(false) {}

InterfaceBLE::InterfaceBLE() : _debug(NULL), configured(false) {}

void InterfaceBLE::serverOnConnectInvCallback() {
    if (serverOnConnectCallback != NULL) {
        serverOnConnectCallback();
    }
}

void InterfaceBLE::serverOnDisconnectInvCallback() {
    if (serverOnDisconnectCallback != NULL) {
        serverOnDisconnectCallback();
    }
}

void InterfaceBLE::charOnWriteInvCallback(std::string uuid, std::string value) {
    if (uuid == ble_uuids->char1 && char1writeCallback != NULL) {
        char1writeCallback(value);
    } else if (uuid == ble_uuids->char2 && char2writeCallback != NULL) {
        char2writeCallback(value);
    } else if (uuid == ble_uuids->char3 && char3writeCallback != NULL) {
        char3writeCallback(value);
    }
}

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        InterfaceBLE::charOnWriteInvCallback(
            pCharacteristic->getUUID().toString(), pCharacteristic->getValue());
    }
};

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        InterfaceBLE::serverOnConnectInvCallback();
    };

    void onDisconnect(BLEServer* pServer) {
        InterfaceBLE::serverOnDisconnectInvCallback();
    };
};

void InterfaceBLE::configure(uint8_t* package, uint8_t package_size,
                             BLE_UUID_Config_t* config_uuids) {
    DEBUG_LN_BLE("Initialization");
    BLEDevice::init("ESP_GATT_SERVER");
    pServer = BLEDevice::createServer();

    if (config_uuids != NULL) {
        ble_uuids = config_uuids;
    }
    packageCharSize = package_size;

    BLEService* pService = pServer->createService(ble_uuids->service);
    DEBUG_BLE("Service UUID: ");
    DEBUG_LN(pService->getUUID().toString().c_str());

    pServer->setCallbacks(new ServerCallbacks());

    pCharacteristic1 = pService->createCharacteristic(
        ble_uuids->char1,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

    pCharacteristic1->setCallbacks(new CharacteristicCallbacks());
    DEBUG_BLE("Characteristic1 UUID: ");
    DEBUG_LN(pCharacteristic1->getUUID().toString().c_str());

    pCharacteristic2 = pService->createCharacteristic(
        ble_uuids->char2, BLECharacteristic::PROPERTY_READ |
                              BLECharacteristic::PROPERTY_WRITE |
                              BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic2->addDescriptor(new BLE2902());

    pCharacteristic2->setCallbacks(new CharacteristicCallbacks());

    DEBUG_BLE("Characteristic2 UUID: ");
    DEBUG_LN(pCharacteristic2->getUUID().toString().c_str());

    pCharacteristic3 = pService->createCharacteristic(
        ble_uuids->char3,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

    pCharacteristic3->setCallbacks(new CharacteristicCallbacks());

    DEBUG_BLE("Characteristic3 UUID: ");
    DEBUG_LN(pCharacteristic3->getUUID().toString().c_str());

    pCharacteristic1->setValue(package, packageCharSize);
    pCharacteristic2->setValue(package, packageCharSize);
    pCharacteristic3->setValue(package, packageCharSize);
    pService->start();

    pAdvertising = pServer->getAdvertising();

    pAdvertising->addServiceUUID(ble_uuids->service);

    configured = true;
}

void InterfaceBLE::setCharacteristicsCallbacks(
    void (*char1write)(std::string), void (*char2write)(std::string),
    void (*char3write)(std::string)) {
    char1writeCallback = char1write;
    char2writeCallback = char2write;
    char3writeCallback = char3write;
}
void InterfaceBLE::setServerCallbacks(void (*onConnect)(void),
                                      void (*onDisconnect)(void)) {
    serverOnConnectCallback = onConnect;
    serverOnDisconnectCallback = onDisconnect;
}

bool InterfaceBLE::StartServiceAdvertising() {
    if (!configured) {
        DEBUG_LN_BLE("Could not start advertising, first configure ble!");
        return false;
    }
    pAdvertising->start();
    return true;
}

bool InterfaceBLE::StopServiceAdvertising() {
    if (!configured) {
        DEBUG_LN_BLE("Could not stop advertising, first configure ble!");
        return false;
    }
    pAdvertising->stop();
    return true;
}

bool InterfaceBLE::sendNotifyChar2(uint8_t* package) {
    if (!configured) {
        DEBUG_LN_BLE("Could not notify Characteristic 2, first configure ble!");
        return false;
    }
    pCharacteristic2->setValue(package, packageCharSize);
    pCharacteristic2->notify();
    return true;
}