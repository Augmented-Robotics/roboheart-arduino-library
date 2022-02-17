#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEAdvertising.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define GATTS_SERVICE_UUID 0x00FF
uint8_t service_uuid[16] = {
  /* LSB <--------------------------------------------------------------------------------> MSB */
  //first uuid, 16bit, [12],[13] is the value
  0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, (uint8_t) (GATTS_SERVICE_UUID), (uint8_t) (GATTS_SERVICE_UUID >> 8), 0x00, 0x00,
};

#define SERVICE_UUID        "000000ff-0000-1000-8000-00805f9b34fb"

//this short String also results in 000000ff-0000-1000-8000-00805f9b34fb:
//#define SERVICE_UUID        "00FF"

#define CHARACTERISTIC_UUID1 "FF01"
#define CHARACTERISTIC_UUID2 "FF02"
#define CHARACTERISTIC_UUID3 "FF03"

static uint8_t char_value[4] = {0x11, 0x22, 0x33, 0x44};

#define ESP_APP_ID                  0x55

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic2 = NULL;


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};

void setup() {
  Serial.begin(115200);

  BLEDevice::init("ESP_GATT_SERVER");
  pServer = BLEDevice::createServer();
  pServer->m_appId = ESP_APP_ID;

  BLEService *pService = pServer->createService(SERVICE_UUID);
  //BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x00FF));         //result: 000000ff-0000-1000-8000-00805f9b34fb
  //BLEService *pService = pServer->createService(BLEUUID(service_uuid, 16, false));//result: 000000ff-0000-1000-8000-00805f9b34fb
  Serial.print("Service UUID: ");
  Serial.println(pService->getUUID().toString().c_str());

  pServer->setCallbacks(new MyServerCallbacks());

  //esp_ble_gatts_app_register(ESP_APP_ID);


  BLECharacteristic *pCharacteristic1 = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID1,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );

  pCharacteristic1->setCallbacks(new MyCallbacks());

  
  pCharacteristic2 = pService->createCharacteristic(
                       CHARACTERISTIC_UUID2,
                       BLECharacteristic::PROPERTY_READ |
                       BLECharacteristic::PROPERTY_WRITE |
                       BLECharacteristic::PROPERTY_NOTIFY
                     );
  pCharacteristic2->addDescriptor(new BLE2902());

  pCharacteristic2->setCallbacks(new MyCallbacks());

  BLECharacteristic *pCharacteristic3 = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID3,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );

  pCharacteristic3->setCallbacks(new MyCallbacks());

  pCharacteristic1->setValue(char_value, 4);
  pCharacteristic2->setValue(char_value, 4);
  pCharacteristic3->setValue(char_value, 4);
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void loop() {
  // notify changed value
  if (deviceConnected) {
    char_value[0]++;
    pCharacteristic2->setValue(char_value, 4);
    pCharacteristic2->notify();
    value++;
    delay(3); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}