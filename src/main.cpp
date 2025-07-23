
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

#define MOUSE_REPORT 0x01
#define KEYBOARD_REPORT 0x02

typedef struct {
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keys[6];
} KeyReport;

static const uint8_t _hidReportDescriptor[] = {
    USAGE_PAGE(1), 0x01, // USAGE_PAGE (Generic Desktop) // for category of device
    USAGE(1), 0x02,      // USAGE (Mouse) // type of device
    COLLECTION(1), 0x01, // COLLECTION (Application)
    USAGE(1), 0x01,      //     USAGE (Pointer)
    COLLECTION(1), 0x00, //     COLLECTION (Physical)
    REPORT_ID(1), 0x01,  //     REPORT_ID (1)

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
    END_COLLECTION(0),        // END_COLLECTION

    
     // Input
    USAGE_PAGE(1),      0x01,                     // USAGE_PAGE (Generic Desktop Ctrls)
    USAGE(1),           0x06,                     // USAGE (Keyboard)
    COLLECTION(1),      0x01,                     // COLLECTION (Application)
    REPORT_ID(1),       0x02,                     //   REPORT_ID (2)
    USAGE_PAGE(1),      0x07,                     //   USAGE_PAGE (Kbrd/Keypad)
    USAGE_MINIMUM(1),   0xE0,                     //   USAGE_MINIMUM (0xE0)
    USAGE_MAXIMUM(1),   0xE7,                     //   USAGE_MAXIMUM (0xE7)
    LOGICAL_MINIMUM(1), 0x00,                     //   LOGICAL_MINIMUM (0)
    LOGICAL_MAXIMUM(1), 0x01,                     //   Logical Maximum (1)
    REPORT_SIZE(1),     0x01,                     //   REPORT_SIZE (1)
    REPORT_COUNT(1),    0x08,                     //   REPORT_COUNT (8)
    HIDINPUT(1),        0x02,                     //   INPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position) ; Modifier byte
    REPORT_COUNT(1),    0x01,                     //   REPORT_COUNT (1) ; 1 byte (Reserved)
    REPORT_SIZE(1),     0x08,                     //   REPORT_SIZE (8)
    HIDINPUT(1),        0x01,                     //   INPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position) ; Reserved byte
    REPORT_COUNT(1),    0x06,                     //   REPORT_COUNT (6) ; 6 bytes (Keys)
    REPORT_SIZE(1),     0x08,                     //   REPORT_SIZE(8)
    LOGICAL_MINIMUM(1), 0x00,                     //   LOGICAL_MINIMUM(0)
    LOGICAL_MAXIMUM(1), 0x65,                     //   LOGICAL_MAXIMUM(0x65) ; 101 keys
    USAGE_PAGE(1),      0x07,                     //   USAGE_PAGE (Kbrd/Keypad)
    USAGE_MINIMUM(1),   0x00,                     //   USAGE_MINIMUM (0)
    USAGE_MAXIMUM(1),   0x65,                     //   USAGE_MAXIMUM (0x65)
    HIDINPUT(1),        0x00,                     //   INPUT (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position) ; Key arrays (6 bytes)
    // Output
    REPORT_COUNT(1),    0x05,                     //   REPORT_COUNT (5) ; 5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
    REPORT_SIZE(1),     0x01,                     //   REPORT_SIZE (1)
    USAGE_PAGE(1),      0x08,                     //   USAGE_PAGE (LEDs)
    USAGE_MINIMUM(1),   0x01,                     //   USAGE_MINIMUM (0x01) ; Num Lock
    USAGE_MAXIMUM(1),   0x05,                     //   USAGE_MAXIMUM (0x05) ; Kana
    HIDOUTPUT(1),       0x02,                     //   OUTPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    REPORT_COUNT(1),    0x01,                     //   REPORT_COUNT (1) ; 3 bits (Padding)
    REPORT_SIZE(1),     0x03,                     //   REPORT_SIZE (3)
    HIDOUTPUT(1),       0x01,                     //   OUTPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    END_COLLECTION(0),           
};

class BleConnectionStatus : public BLEServerCallbacks
{
public:
    BLECharacteristic *inputMouse;
    BLECharacteristic *inputKeyboard;
    bool connected = false;

    BleConnectionStatus(void) {};

    void onConnect(BLEServer *pServer)
    {
        this->connected = true;
        BLE2902 *desc = (BLE2902 *)this->inputKeyboard->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        desc->setNotifications(true);
        desc = (BLE2902 *)this->inputMouse->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        desc->setNotifications(true);
    };

    void onDisconnect(BLEServer *pServer)
    {
        this->connected = false;
        BLE2902 *desc = (BLE2902 *)this->inputKeyboard->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        desc->setNotifications(false);
        desc = (BLE2902 *)this->inputMouse->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        desc->setNotifications(false);
    };
};

BLECharacteristic *mouse;
BLECharacteristic *keyboard;

BleConnectionStatus *connectionStatus = new BleConnectionStatus();

void taskServer(void)
{
    BLEDevice::init("WTF IS A KILOMETER!?");
    BLEServer *pServer = BLEDevice::createServer();

    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_BOND_MITM);

    BLEHIDDevice hid(pServer);

    mouse = hid.inputReport(1);
    keyboard = hid.inputReport(2);
    connectionStatus->inputMouse = mouse;
    connectionStatus->inputKeyboard = keyboard;
    pServer->setCallbacks(connectionStatus);

    hid.pnp(0x02, 0xe502, 0xa111, 0x0210);
    hid.hidInfo(0x00, 0x02);
    hid.manufacturer()->setValue("AleksGoncharuk");

    hid.reportMap((uint8_t *)_hidReportDescriptor, sizeof(_hidReportDescriptor));
    hid.startServices();

    hid.setBatteryLevel(30);

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setAppearance(GENERIC_HID);
    pAdvertising->addServiceUUID(hid.hidService()->getUUID());
    // pAdvertising->setScanResponse(false);
    pAdvertising->start();

    vTaskDelay(portMAX_DELAY);
}

void mouseSend(uint8_t buttons, int x, int y, int wheel, int hWheel)
{
    if (connectionStatus->connected)
    {
        uint8_t mouseState[5];
        // mouseState[0] = MOUSE_REPORT;
        mouseState[0] = buttons;
        mouseState[1] = x;
        mouseState[2] = y;
        mouseState[3] = wheel;
        mouseState[4] = hWheel;

        mouse->setValue(mouseState, 5);
        mouse->notify();
    }
}

void keyboardSend(uint8_t modifier, uint8_t key)
{
    if (connectionStatus->connected)
    {
        Serial.println("KeyboardSend");
        KeyReport report;
        report.modifiers = modifier;
        report.reserved = 0x00;
        report.keys[0] = key;
        for (int i = 1; i < 6; i++) {
            report.keys[i] = 0x00;
        }
        keyboard->setValue((uint8_t *)&report, sizeof(report));
        keyboard->notify();
    }
}

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

    mouseSend(MOUSE_NONE, 3, 3, 0, 0);


    if (digitalRead(9) == LOW) {
        keyboardSend(0x08, 0x00);
        delay(10);
        keyboardSend(0x00, 0x00);
    }

    digitalWrite(8, LOW);

    Serial.println(temperatureRead());
    digitalWrite(8, HIGH);
}
