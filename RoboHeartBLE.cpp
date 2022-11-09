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

#define FILE_IDENTIFIER \
    "BLE"  // Define identifier before including DebuggerMsgs.h
#include "DebuggerMsgs.h"

static uuidConfigType uuids = {RH_APP_SERVICE_UUID, RH_APP_CHARACTERISTIC_UUID1,
                               RH_APP_CHARACTERISTIC_UUID2,
                               RH_APP_CHARACTERISTIC_UUID3};

static uint8_t bleDefaultPackage[4] = {0x11, 0x22, 0x33, 0x44};

uuidConfigType* InterfaceBLE::_uuidsBLE = &uuids;

void (*InterfaceBLE::char1WriteCallback)(std::string) = NULL;
void (*InterfaceBLE::char2WriteCallback)(std::string) = NULL;
void (*InterfaceBLE::char3WriteCallback)(std::string) = NULL;
void (*InterfaceBLE::serverOnConnectCallback)(void) = NULL;
void (*InterfaceBLE::serverOnDisconnectCallback)(void) = NULL;

InterfaceBLE::InterfaceBLE(Stream& debug) : _debug(&debug) {}

InterfaceBLE::InterfaceBLE() {}

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
    if (uuid == _uuidsBLE->char1 && char1WriteCallback != NULL) {
        char1WriteCallback(value);
    } else if (uuid == _uuidsBLE->char2 && char2WriteCallback != NULL) {
        char2WriteCallback(value);
    } else if (uuid == _uuidsBLE->char3 && char3WriteCallback != NULL) {
        char3WriteCallback(value);
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

void InterfaceBLE::begin(uint8_t* package, uint8_t packageSize,
                             uuidConfigType* uuidsConfig) {
    DEBUG_LN_IDENTIFIER("Initialization");
    BLEDevice::init("ESP_GATT_SERVER_RH");
    _Server = BLEDevice::createServer();

    if (uuidsConfig != NULL) {
        _uuidsBLE = uuidsConfig;
    }

    if (package == NULL){
        package = bleDefaultPackage;
        _packageCharSize = sizeof(bleDefaultPackage);
    }

    BLEService* pService = _Server->createService(_uuidsBLE->service);
    DEBUG_IDENTIFIER("Service UUID: ");
    DEBUG_LN(pService->getUUID().toString().c_str());

    _Server->setCallbacks(new ServerCallbacks());

    _characteristic1 = pService->createCharacteristic(
        _uuidsBLE->char1,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

    _characteristic1->setCallbacks(new CharacteristicCallbacks());
    DEBUG_IDENTIFIER("Characteristic1 UUID: ");
    DEBUG_LN(_characteristic1->getUUID().toString().c_str());

    _characteristic2 = pService->createCharacteristic(
        _uuidsBLE->char2, BLECharacteristic::PROPERTY_READ |
                              BLECharacteristic::PROPERTY_WRITE |
                              BLECharacteristic::PROPERTY_NOTIFY);
    _characteristic2->addDescriptor(new BLE2902());

    _characteristic2->setCallbacks(new CharacteristicCallbacks());

    DEBUG_IDENTIFIER("Characteristic2 UUID: ");
    DEBUG_LN(_characteristic2->getUUID().toString().c_str());

    _characteristic3 = pService->createCharacteristic(
        _uuidsBLE->char3,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

    _characteristic3->setCallbacks(new CharacteristicCallbacks());

    DEBUG_IDENTIFIER("Characteristic3 UUID: ");
    DEBUG_LN(_characteristic3->getUUID().toString().c_str());

    _characteristic1->setValue(package, _packageCharSize);
    _characteristic2->setValue(package, _packageCharSize);
    _characteristic3->setValue(package, _packageCharSize);
    pService->start();

    _Advertising = _Server->getAdvertising();

    _Advertising->addServiceUUID(_uuidsBLE->service);

    _configured = true;
}

void InterfaceBLE::setCharacteristicsCallbacks(
    void (*char1Write)(std::string), void (*char2Write)(std::string),
    void (*char3Write)(std::string)) {
    char1WriteCallback = char1Write;
    char2WriteCallback = char2Write;
    char3WriteCallback = char3Write;
}

void InterfaceBLE::setServerCallbacks(void (*onConnect)(void),
                                      void (*onDisconnect)(void)) {
    serverOnConnectCallback = onConnect;
    serverOnDisconnectCallback = onDisconnect;
}

bool InterfaceBLE::startServiceAdvertising() {
    RETURN_VAL_WARN_IF_EQUAL(_configured, false, false)

    _Advertising->start();
    return true;
}

bool InterfaceBLE::stopServiceAdvertising() {
    RETURN_VAL_WARN_IF_EQUAL(_configured, false, false)

    _Advertising->stop();
    return true;
}

bool InterfaceBLE::sendNotifyChar2(uint8_t* package) {
    RETURN_VAL_WARN_IF_EQUAL(_configured, false, false)
    
    _characteristic2->setValue(package, _packageCharSize);
    _characteristic2->notify();
    return true;
}
bool InterfaceBLE::sendNotifyChar3(uint8_t* package){
    RETURN_VAL_WARN_IF_EQUAL(_configured, false, false)
    
    _characteristic3->setValue(package, _packageCharSize);
    return true;
}