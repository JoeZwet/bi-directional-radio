#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>

#pragma region LibrarySetup

RF24 radio(9, 10);                  // (CSN, CE)
LiquidCrystal_I2C lcd(0x3f, 16, 2); //0x27 or 0x3F, width, height
Servo srv;

#pragma endregion

#pragma region Pins

int x = A0; // joystick x
int y = A1; // joystick y
int b = 2;  // joystick button
int p = A1; // potentiometer
int m = 3;  // motor
int l = 7;  // led
int s = A3; // servo

#pragma endregion

#pragma region Data

const byte addr[][6] = {"00003", "00004"}; // radio addresses
const bool isDisplaySide = false;          // side with lcd display, confirm before uploading
volatile bool state = false;                        // button state
struct Data
{
    int x;  // joystick x
    int y;  // joystick y
    bool b; // joystick button
};

volatile int r; // cache reset/potentiometer pos
volatile Data data; // cache data

#pragma endregion

#pragma region Setup

void setup()
{
    // general setup
    Serial.begin(9600);
    radio.begin();
    radio.enableAckPayload();
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MIN);

    if (isDisplaySide)
    {
        setup_display_side();
    }
    else
    {
        setup_motor_side();
    }
}

// setup side with display
void setup_display_side()
{
    // setup radio
    radio.openWritingPipe(addr[0]);

    // setup lcd
    lcd.init();
    lcd.backlight();
    lcd.clear();
    write_lcd_labels();

    // setup lcd timer interrupt
    Timer1.initialize(100); // interrupt every 100ms
    Timer1.attachInterrupt(write_lcd_values); // run write_lcd_values() every interrupt

    // setup joystick
    pinMode(x, INPUT);
    pinMode(y, INPUT);
    pinMode(b, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(b), handle_button_interrupt, FALLING); // attach an interrupt to the falling edge of the joystick button
}

// setup side with motor
void setup_motor_side()
{
    // setup radio
    radio.openReadingPipe(1, addr[0]);
    radio.startListening();

    // setup servo
    srv.attach(s);

    // setup led
    pinMode(l, OUTPUT);

    // setup motor
    pinMode(m, OUTPUT);

    // setup potentiometer
    pinMode(p, INPUT);
}

#pragma endregion

#pragma region Loop

void loop()
{
    if (isDisplaySide)
    {
        loop_display_side();
    }
    else
    {
        loop_motor_side();
    }
}

void loop_display_side()
{
    if (send_data()) // send data on radio
    {
        if (radio.isAckPayloadAvailable()) // check if ack payload is avaliable
        {
            radio.read(&r, sizeof(r)); // read ack payload
        }
    }
}

void loop_motor_side()
{
    if (radio.available()) // check if data is avaliable to be read
    {
        radio.read(&data, sizeof(Data)); // read data from radio
        send_ack(); // set ack payload
    }

    if (data.b)
    {                          // reset is toggled on
        analogWrite(m, 0);     // set motor speed to 0
        digitalWrite(l, HIGH); // turn on led
        srv.write(r);          // set servo to reset/potentiometer pos
    }
    else
    {                           // reset is toggled off, normal operation
        analogWrite(m, data.y); // write y value to motor speed
        digitalWrite(l, LOW);   // turn off led
        srv.write(data.x);      // write x value to servo
    }
}

#pragma endregion

#pragma region Utilities

bool send_data()
{
    data.x = map(analogRead(x), 0, 1023, 0, 180); // read and map joystick x value
    data.y = map(analogRead(y), 0, 1023, 0, 255); // read and map joystick y value
    data.b = state;

    return radio.write(&data, sizeof(Data), 0); // write data to radio
}

bool handle_button_interrupt()
{
    state = !state; // flip state
}

void send_ack()
{
    r = map(analogRead(p), 0, 1023, 0, 180); // read and map potentiometer value
    radio.writeAckPayload(1, &r, sizeof(r)); // write the reset value to the ack payload
}

void write_lcd_labels()
{
    lcd.setCursor(0, 0);
    lcd.print("svr:"); // write servo pos label
    lcd.setCursor(8, 0);
    lcd.print("spd:"); // write motor speed label
    lcd.setCursor(0, 1);
    lcd.print("btn:"); // write button label
    lcd.setCursor(8, 1);
    lcd.print("rst:"); // write reset label
}

void write_lcd_values()
{
    lcd.setCursor(4, 0);
    lcd.print("    "); // clear servo value area
    lcd.setCursor(4, 0);
    lcd.print(String(data.x)); // write sero pos value
    lcd.setCursor(12, 0);
    lcd.print("    "); // clear motor value area
    lcd.setCursor(12, 0);
    lcd.print(String(data.y)); // write motor speed value
    lcd.setCursor(4, 1);
    lcd.print(data.b ? "ON " : "OFF"); // write button value
    lcd.setCursor(12, 1);
    lcd.print("    "); // clear reset value area
    lcd.setCursor(12, 1);
    lcd.print(String(r)); // write reset value
}

#pragma endregion