#include <Arduino.h>
#include <math.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"
#include "HIDKeyboardTypes.h"

#include <MPU6050_6Axis_MotionApps20.h>

#define MOUSE_NONE 0b00000
#define MOUSE_LEFT 0b00001
#define MOUSE_RIGHT 0b00010
#define MOUSE_MIDDLE 0b00100

#define MOUSE_REPORT 0x01
#define KEYBOARD_REPORT 0x02

#define ENCODER_A 0
#define ENCODER_B 1


typedef struct
{
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keys[6];
} KeyReport;

static const uint8_t _hidReportDescriptor[] = {

    // Input
    USAGE_PAGE(1), 0x01,      // USAGE_PAGE (Generic Desktop Ctrls)
    USAGE(1), 0x06,           // USAGE (Keyboard)
    COLLECTION(1), 0x01,      // COLLECTION (Application)
    REPORT_ID(1), 0x01,       //   REPORT_ID (2)
    USAGE_PAGE(1), 0x07,      //   USAGE_PAGE (Kbrd/Keypad)
    USAGE_MINIMUM(1), 0xE0,   //   USAGE_MINIMUM (0xE0)
    USAGE_MAXIMUM(1), 0xE7,   //   USAGE_MAXIMUM (0xE7)
    LOGICAL_MINIMUM(1), 0x00, //   LOGICAL_MINIMUM (0)
    LOGICAL_MAXIMUM(1), 0x01, //   Logical Maximum (1)
    REPORT_SIZE(1), 0x01,     //   REPORT_SIZE (1)
    REPORT_COUNT(1), 0x08,    //   REPORT_COUNT (8)
    HIDINPUT(1), 0x02,        //   INPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position) ; Modifier byte
    REPORT_COUNT(1), 0x01,    //   REPORT_COUNT (1) ; 1 byte (Reserved)
    REPORT_SIZE(1), 0x08,     //   REPORT_SIZE (8)
    HIDINPUT(1), 0x01,        //   INPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position) ; Reserved byte
    REPORT_COUNT(1), 0x06,    //   REPORT_COUNT (6) ; 6 bytes (Keys)
    REPORT_SIZE(1), 0x08,     //   REPORT_SIZE(8)
    LOGICAL_MINIMUM(1), 0x00, //   LOGICAL_MINIMUM(0)
    LOGICAL_MAXIMUM(1), 0x65, //   LOGICAL_MAXIMUM(0x65) ; 101 keys
    USAGE_PAGE(1), 0x07,      //   USAGE_PAGE (Kbrd/Keypad)
    USAGE_MINIMUM(1), 0x00,   //   USAGE_MINIMUM (0)
    USAGE_MAXIMUM(1), 0x65,   //   USAGE_MAXIMUM (0x65)
    HIDINPUT(1), 0x00,        //   INPUT (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position) ; Key arrays (6 bytes)
    // Output
    REPORT_COUNT(1), 0x05,  //   REPORT_COUNT (5) ; 5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
    REPORT_SIZE(1), 0x01,   //   REPORT_SIZE (1)
    USAGE_PAGE(1), 0x08,    //   USAGE_PAGE (LEDs)
    USAGE_MINIMUM(1), 0x01, //   USAGE_MINIMUM (0x01) ; Num Lock
    USAGE_MAXIMUM(1), 0x05, //   USAGE_MAXIMUM (0x05) ; Kana
    HIDOUTPUT(1), 0x02,     //   OUTPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    REPORT_COUNT(1), 0x01,  //   REPORT_COUNT (1) ; 3 bits (Padding)
    REPORT_SIZE(1), 0x03,   //   REPORT_SIZE (3)
    HIDOUTPUT(1), 0x01,     //   OUTPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    END_COLLECTION(0),

    USAGE_PAGE(1), 0x01, // USAGE_PAGE (Generic Desktop) // for category of device
    USAGE(1), 0x02,      // USAGE (Mouse) // type of device
    COLLECTION(1), 0x01, // COLLECTION (Application)
    USAGE(1), 0x01,      //     USAGE (Pointer)
    COLLECTION(1), 0x00, //     COLLECTION (Physical)
    REPORT_ID(1), 0x02,  //     REPORT_ID (1)

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

class Button
{
private:
    int pin; // Цифровий порт, до якого підключена кнопка
    int actionThreshold;
    int holdThreshold;

    bool clicked = false; // Чи кнопка натиснута (тільк перший раз)
    bool hold = false;    // Чи кнопка затиснута
    bool pressed = false; // Чи кнопка натиснута (взагалі)
    int pressedTime = 0;  // Час з початку натиску кнопки в мілісекундах

    int action = LOW;       // Сигнал з цифрового порта
    int lastAction = LOW;   // Минулий сигнал з цифрового порта
    int lastActionTime = 0; // Час минулої зміни сигналу з цифрового порта

public:
    // Конструктор. Приймає цифровий порт, до якого приєднана кнопка
    Button(int pin, int actionThreshold, int holdThreshold)
    {
        this->pin = pin;
        this->actionThreshold = actionThreshold;
        this->holdThreshold = holdThreshold;
    };

    // Ініціалізація кнопки шляхом встановлення режиму роботи порта на вхід\зчитування
    void init()
    {
        pinMode(pin, INPUT);
    }

    // Оновлення данних про кнопку
    void update(int time)
    {
        // Читання поточного сигналу з цифрового порта
        action = digitalRead(this->pin);

        // Якщо сигнал відрізняється від попереднього, то оновити час останої дії
        // кнопки та останього сигналу.
        if (action != lastAction)
        {
            lastAction = action;
            lastActionTime = time;
        }

        // Якщо поточний сигнал високий (тобто кнопка затиснута) і час з останьої дії
        // є більшим за час для зарахування натиску
        if (action == HIGH && time - lastActionTime >= this->actionThreshold)
        {

            // Якщо це вже друга обробка, то кажемо що кнопка вже затиснута, а не натиснута
            // перевіряти коли кнопка була натиснута, а не затиснута
            if (this->hold == false && this->pressed == true && this->pressedTime >= this->holdThreshold)
            {
                this->hold = true;
            }
            // Перевіряємо чи це перша обробка, для того щоб мати змогу
            if (this->pressed == false && this->clicked == false)
            {
                this->clicked = true;
                this->hold = false;
            }
            else if (this-> pressed == true && this->clicked == true) {
                this->clicked = false;
            }

            // Кажемо, що кнопка взагалі натиснута (не важливо чи затиснута чи натиснута вперше)
            this->pressed = true;
        }
        // Якщо ні, то кажемо що кнопка взагалі не натиснута і час натиску відповідно
        // дорівнює 0
        else
        {
            this->pressed = false;
            this->pressedTime = 0;
            this->hold = false;
            this->clicked = false;
        }

        // Якщо конпка взагалі натиснута, то відслідковуємо час скільки вона є натиснутою
        if (this->pressed)
        {
            pressedTime = time - lastActionTime;
        }
    }

    // Методи-інтерфейс для отримання данних з кнопки

    bool getPressed()
    {
        return this->pressed;
    }

    int getPressedTime()
    {
        return this->pressedTime;
    }

    bool getClicked()
    {
        return this->clicked;
    }

    bool getHold()
    {
        return this->hold;
    }

    int getPin() {
        return this->pin;
    }

    // Скидання стану кнопки, крім стану натиску взагалі. Використовується для
    // того, щоб, наприклад, коли користувач затиснув на вихід в меню його одразу не
    // перекинуло назад в меню (якщо стан затиску і час натиску залишиться такими ж,
    // тоді програма подумає що користувач спеціально затиснув кнопку на екрані годиника
    // і хоче в меню)
    void reset()
    {
        this->pressedTime = 0;
        this->hold = false;
        this->clicked = false;
    }
};

BLECharacteristic *mouse;
BLECharacteristic *keyboard;

BleConnectionStatus *connectionStatus = new BleConnectionStatus();

MPU6050 mpu;

VectorFloat gravity;
Quaternion quaternion;
float ypr[3], last_ypr[3];
uint8_t FIFOBuffer[64];

int wheelAction, lastWheelAction;
int currentTime, lastTime;

Button leftButton(3, 100, 250);
Button rightButton(2, 100, 250);

void taskServer(void)
{
    BLEDevice::init("WTF IS A KILOMETER?!");
    BLEServer *pServer = BLEDevice::createServer();

    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

    BLEHIDDevice hid(pServer);

    keyboard = hid.inputReport(1);
    mouse = hid.inputReport(2);
    connectionStatus->inputKeyboard = keyboard;
    connectionStatus->inputMouse = mouse;
    pServer->setCallbacks(connectionStatus);

    hid.pnp(0x02, 0xe502, 0xa111, 0x0210);
    hid.hidInfo(0x00, 0x02);
    hid.manufacturer()->setValue("AleksGoncharuk");

    hid.reportMap((uint8_t *)_hidReportDescriptor, sizeof(_hidReportDescriptor));
    hid.startServices();

    hid.setBatteryLevel(99);

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setAppearance(ESP_BLE_APPEARANCE_GENERIC_REMOTE);
    pAdvertising->addServiceUUID(hid.hidService()->getUUID());
    pAdvertising->setScanResponse(false);
    pAdvertising->start();

    vTaskDelay(portMAX_DELAY);
}

void mouseSend(uint8_t buttons, int x, int y, int wheel, int hWheel)
{
    if (connectionStatus->connected)
    {
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

void keyboardSend(uint8_t modifier, uint8_t key)
{
    if (connectionStatus->connected)
    {
        KeyReport report;
        report.modifiers = modifier;
        report.reserved = 0x00;
        report.keys[0] = key;
        for (int i = 1; i < 6; i++)
        {
            report.keys[i] = 0x00;
        }
        keyboard->setValue((uint8_t *)&report, sizeof(report));
        keyboard->notify();
    }
}

void setup()
{
    pinMode(8, OUTPUT);
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    digitalWrite(8, LOW);

    Wire.begin(20, 21);
    Wire.setClock(400000);

    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXGyroOffset(121);
    mpu.setYGyroOffset(-147);
    mpu.setZGyroOffset(-30);
    mpu.setXAccelOffset(4698);
    mpu.setYAccelOffset(-3243);
    mpu.setZAccelOffset(12798);
    mpu.setDMPEnabled(true);

    xTaskCreate((TaskFunction_t)taskServer, "server", 20000, NULL, 5, NULL);

    mpu.dmpGetCurrentFIFOPacket(FIFOBuffer);
    mpu.dmpGetQuaternion(&quaternion, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &quaternion);
    mpu.dmpGetYawPitchRoll(last_ypr, &quaternion, &gravity);

    lastWheelAction = 0;
    lastTime = millis();
}

void loop()
{
    currentTime = millis();

    if (currentTime - lastTime >= 10) {
        digitalWrite(8, LOW);

        int offsetX = 0, offsetY = 0, wheel = 0;
    
        leftButton.update(currentTime);
        rightButton.update(currentTime);

        wheelAction = digitalRead(ENCODER_A);
        if (wheelAction != lastWheelAction && wheelAction == HIGH)
        {
            wheel = digitalRead(ENCODER_B) ? 1 : -1;
    
        }
        lastWheelAction = wheelAction;
    
        if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer))
        {
            mpu.dmpGetQuaternion(&quaternion, FIFOBuffer);
            mpu.dmpGetGravity(&gravity, &quaternion);
            mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
    
            offsetX = (ypr[0] - last_ypr[0]) * 180/PI * 20;
            offsetY = -(ypr[2] - last_ypr[2]) * 180/PI * 20;

            for (int i = 0; i < 3; i++) {
                last_ypr[i] = ypr[i];
            }
        }

        uint8_t leftAction = leftButton.getClicked() or leftButton.getHold() ? MOUSE_LEFT : MOUSE_NONE;
        uint8_t rightAction = rightButton.getClicked() or rightButton.getHold() ? MOUSE_RIGHT : MOUSE_NONE;

        mouseSend(leftAction | rightAction, offsetX, offsetY, wheel, 0);
        digitalWrite(8, HIGH);
        lastTime = currentTime;
    }

}
