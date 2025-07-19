
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"
#include "HIDKeyboardTypes.h"

#define MOUSE_NONE 0b00000
#define MOUSE_LEFT 0b00001
#define MOUSE_RIGHT 0b00010
#define MOUSE_MIDDLE 0b00100


static const uint8_t _hidReportDescriptor[] = {
    USAGE_PAGE(1), 0x01, // USAGE_PAGE (Generic Desktop) // for category of device
    USAGE(1), 0x02,      // USAGE (Mouse) // type of device
    COLLECTION(1), 0x01, // COLLECTION (Application)
    USAGE(1), 0x01,      //   USAGE (Pointer)
    COLLECTION(1), 0x00, //   COLLECTION (Physical)
    REPORT_ID(1), 0x00, // REPORT_ID (0)

    // ------------------------------------------------- Buttons (Left, Right, Middle, Back, Forward)
    USAGE_PAGE(1), 0x09,      //     USAGE_PAGE (Button)
    USAGE_MINIMUM(1), 0x01,   //     USAGE_MINIMUM (Button 1)
    USAGE_MAXIMUM(1), 0x05,   //     USAGE_MAXIMUM (Button 5)
    LOGICAL_MINIMUM(1), 0x00, //     LOGICAL_MINIMUM (0)
    LOGICAL_MAXIMUM(1), 0x01, //     LOGICAL_MAXIMUM (1)
    REPORT_SIZE(1), 0x01,     //     REPORT_SIZE (1)
    REPORT_COUNT(1), 0x05,    //     REPORT_COUNT (5)
    HIDINPUT(1), 0x02,        //     INPUT (Data, Variable, Absolute) ;5 button bits
    // ------------------------------------------------- Padding
    REPORT_SIZE(1), 0x03,  //     REPORT_SIZE (3)
    REPORT_COUNT(1), 0x01, //     REPORT_COUNT (1)
    HIDINPUT(1), 0x03,     //     INPUT (Constant, Variable, Absolute) ;3 bit padding
    // ------------------------------------------------- X/Y position, Wheel
    USAGE_PAGE(1), 0x01,      //     USAGE_PAGE (Generic Desktop)
    USAGE(1), 0x30,           //     USAGE (X)
    USAGE(1), 0x31,           //     USAGE (Y)
    USAGE(1), 0x38,           //     USAGE (Wheel)
    LOGICAL_MINIMUM(1), 0x81, //     LOGICAL_MINIMUM (-127)
    LOGICAL_MAXIMUM(1), 0x7f, //     LOGICAL_MAXIMUM (127)
    REPORT_SIZE(1), 0x08,     //     REPORT_SIZE (8)
    REPORT_COUNT(1), 0x03,    //     REPORT_COUNT (3)
    HIDINPUT(1), 0x06,        //     INPUT (Data, Variable, Relative) ;3 bytes (X,Y,Wheel)
    // ------------------------------------------------- Horizontal wheel
    USAGE_PAGE(1), 0x0c,      //     USAGE PAGE (Consumer Devices)
    USAGE(2), 0x38, 0x02,     //     USAGE (AC Pan)
    LOGICAL_MINIMUM(1), 0x81, //     LOGICAL_MINIMUM (-127)
    LOGICAL_MAXIMUM(1), 0x7f, //     LOGICAL_MAXIMUM (127)
    REPORT_SIZE(1), 0x08,     //     REPORT_SIZE (8)
    REPORT_COUNT(1), 0x01,    //     REPORT_COUNT (1)
    HIDINPUT(1), 0x06,        //     INPUT (Data, Var, Rel)
    END_COLLECTION(0),        //   END_COLLECTION
    END_COLLECTION(0),         // END_COLLECTION
    
    // USAGE_PAGE(1), 0x01, //     USAGE_PAGE (Generic Desktop)
    // USAGE(1), 0x06, //          USAGE (Keyboard)
    // COLLECTION(1), 0x01, //     COLLECTION (Application)
    // REPORT_ID(1), 0x01, //          REPORT_ID (1)
    // USAGE_PAGE(1), 0x07, //         USAGE_PAGE (Keyboard/Keypad)
    // USAGE_MINIMUM(1), 0x04, //      USAGE_MINIMUM ("A" Key)
    // USAGE_MAXIMUM(1), 0x04, //      USAGE_MINIMUM ("A" Key)
    // LOGICAL_MINIMUM(1), 0x00, //    LOGICAL_MINIMUM (0)
    // LOGICAL_MAXIMUM(1), 0x01, //    LOGICAL_MAXIMUM (1)
    // REPORT_SIZE(1), 0x01, //        REPORT_SIZE (1)
    // REPORT_COUNT(1), 0x01, //       REPORT_COUNT (1)
    // HIDINPUT(1), 0b010, //          INPUT (Data, Variable, Absolute) 1 key bit
    // REPORT_SIZE(1), 0x07, //        REPORT_SIZE (7)
    // REPORT_COUNT(1), 0x01, //       REPORT_COUNT (1)
    // HIDINPUT(1), 0b011, //          INPUT (Constant, Variable, Absolute) 7 bit padding
    // END_COLLECTION(0),  //      END_COLLECTION
};

class BleConnectionStatus : public BLEServerCallbacks
{
public:
  BleConnectionStatus(void);
  bool connected = false;
  void onConnect(BLEServer* pServer);
  void onDisconnect(BLEServer* pServer);
  BLECharacteristic* inputDevice;
};

BleConnectionStatus::BleConnectionStatus(void) {
}

void BleConnectionStatus::onConnect(BLEServer* pServer)
{
  this->connected = true;
  BLE2902* desc = (BLE2902*)this->inputDevice->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
  desc->setNotifications(true);
}

void BleConnectionStatus::onDisconnect(BLEServer* pServer)
{
  this->connected = false;
  BLE2902* desc = (BLE2902*)this->inputDevice->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
  desc->setNotifications(false);
}


BLECharacteristic *mouse;
BLECharacteristic *keyboard;

BleConnectionStatus *connectionStatus = new BleConnectionStatus();

void taskServer(void)
{
    BLEDevice::init("WTF IS A KILOMETER!?");
    BLEServer *pServer = BLEDevice::createServer();
    
    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);
    
    BLEHIDDevice hid(pServer);
    
    mouse = hid.inputReport(0);
    connectionStatus->inputDevice = mouse;
    pServer->setCallbacks(connectionStatus);

    // keyboard = hid.inputReport(1);

    hid.pnp(0x02, 0xe502, 0xa111, 0x0210);
    hid.hidInfo(0x00, 0x02);
    hid.manufacturer()->setValue("AleksGoncharuk");

    hid.reportMap((uint8_t *)_hidReportDescriptor, sizeof(_hidReportDescriptor));
    hid.startServices();

    hid.setBatteryLevel(69);

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setAppearance(HID_MOUSE);
    pAdvertising->addServiceUUID(hid.hidService()->getUUID());
    pAdvertising->start();

    vTaskDelay(portMAX_DELAY);
}

void mouseSend(uint8_t buttons, int x, int y, int wheel, int hWheel) {
    if (connectionStatus->connected) {
        uint8_t mouseState[5];
        mouseState[0] = buttons;
        mouseState[1] = x;
        mouseState[2] = y;
        mouseState[3] = wheel;
        mouseState[4] = hWheel;

        mouse->setValue(mouseState, 5);
        mouse->notify();
    }
}

// void keyboardSend(uint8_t state) {
//     if (connectionStatus->connected) {
//         uint8_t keyState[1] = {state};
//         keyboard->setValue(keyState, 1);
//         keyboard->notify();
//     }
// }

void setup()
{
    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);

    delay(1000);
    Serial.begin(115200);
    Serial.println("Starting BLE work!");

    xTaskCreate((TaskFunction_t)taskServer, "server", 20000, NULL, 5, NULL);

    // BLEDevice::startAdvertising();
    Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop()
{
    // put your main code here, to run repeatedly:
    delay(200);

    mouseSend(MOUSE_NONE, 1, 1, 0, 0);
    // keyboardSend(0x01);

    digitalWrite(8, LOW);

    Serial.println(temperatureRead());
    digitalWrite(8, HIGH);
}
