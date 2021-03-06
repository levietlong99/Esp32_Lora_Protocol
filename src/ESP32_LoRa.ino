
/*
   #PROJECT: Using esp32 to communicate with firebase, communicate with Arduino Uno through RF (LoRa module).

   ESP32 side: pull control data and push sensor data to firebase, communicate with Arduino Uno to
   send control value and take sensor value in real time.

   #OUTSTANDING FEATURE
   - Using 2 CORE that ESP32 equipped, running paralell (mini RTOS), so missing data will never be happend !
   - Good arrange process for transmit and receive, so process is very sepreate.
   - High speed response, 1 seconds and relay under your controller.
   - Smart arrange process on dual core, so CRASH will never happend on your ESP.
   - Safe? but we still prepared in case bad circumstance happended.

   #NOTICE
   -> Read README.md for more information about project like: circuit, refer sites, ability,...

      Created 8 Janaruary 2021
      by Viet Long
      Contact to my email: vietlongle2000@gmail.com for more information.
*/
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define WIFI_USERNAME   "100.000USDorLoser"
#define WIFI_PASSWORD   "huylong1999"
#define FIREBASE_HOST   "https://loraesp32vippro-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH   "cYhdV1RqRExQSAt0TcHgd8L2WCDyIHc3vnxEgIBd"
/* defines the pins used by the tranceiver module */
#define PIN_SCK         18
#define PIN_MISO        19
#define PIN_MOSI        23
#define PIN_SS          5
#define PIN_RST         14
#define PIN_DI0         26                                              // in my experience, this's the best interrupt pin on ESP32

#define FIREBASE_SENSOR_PATH     "/path_2/sensor"
#define FIREBASE_CONTROL_PATH "/path_1/control"
#define FIREBASE_STATUS_PATH     "/path_3/control_status"

#define RF_BAND         433E6                                           // Asia - Viet Nam

#define LCD_I2C_ADDRESS 0x27
#define LCD_ROWS        4                                               // LCD size
#define LCD_COLUMNS     20
/* store relay state (in case hard reset) */
#define EEPROM_SIZE     1                                               // use 1 byte of EEPROM
#define EEPROM_ADDRESS  0x00

void connect_eps32_wifi();

/* lora function */
void init_lora();
void send_byte_lora(uint8_t package);
void loRa_on_receive_interrupt(int packetLength);

/* lcd function */
void lcd_setup();
void lcd_print_relay_state(uint8_t relay_byte);
void lcd_print_sensor_value(String *sensor_array);

/* analyse data, check data */
bool analyse_control_data(uint8_t control_byte, uint8_t *state_byte);
bool analyse_sensor_string(String *string_array, String raw);
bool wait_for(unsigned long interval);
bool wait_for_door(unsigned long interval);

/* Define FirebaseESP32 data object for data sending and receiving */
FirebaseData fbdo;
/* Lcd initialize */
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);

volatile bool is_new_data = false;                                      // announce: new  completed data ready
volatile bool is_receive_interrupt{false};                              // announce: receive interrupt has occured
String raw_string = "";                                                 // string was  used for storing receive data
uint8_t relay_state = 0;                                                // store relay state
bool is_door_open = false;


void setup() {

    delay(1000);
    // Serial.begin(115200);
    connect_eps32_wifi();

    EEPROM.begin(EEPROM_SIZE);                                          // get relays's state from EEPROM
    relay_state = (EEPROM.read(EEPROM_ADDRESS)) & 0x1F;
    init_lora();
    lcd_setup();
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    /* Uncomment below line to enable syncworld - avoid signal from other LoRa*/
    // LoRa.setSyncWord(0xF3);

    Firebase.reconnectWiFi(true);                                       // enable auto reconnect the WiFi when connection lost
    lcd_print_relay_state(relay_state);
    send_byte_lora(relay_state);                                        // send new relay state to Node
    /* Syntax to creat loop on another core , run paralell with main loop */
    xTaskCreatePinnedToCore(

        loop_run_in_core_0,                                             // loop function run in core 0
        "loop_run_in_core_0",                                           // name of the task (for debugging)
        5000,                                                           // stack size for core 0 (bytes)
        NULL,                                                           // paramester to pass in loop
        1,                                                              // task priority (set this to one so we dont miss any data)
        NULL,                                                           // task handle (just run loop 0)
        0                                                               // core that the task run on
    );
}

/* LOOP RUN ON CORE 1, (COMMUNICATE WITH FIREBASE, RECEIVE DATA FROM NODE THROUGH LoRa) ******************************/
void loop() {

    yield();                                                            // give time for CPU complete it's work
    if(is_receive_interrupt){

        return;                                                         // if data is arriving, return and wait for core 0
    }
    else{

        /* NEW SENSOR DATA RECEIVED FROM NODE, CHECK AND PUSH IT TO FIREBASE ********************/

        if(is_new_data) {                                               // check GLOBAL bool if new data was completely received

            String sensor_array[3];
            // Serial.println(raw_string);
            /* check if data don't include noise, if not, fill sensor data to sensor_array*/
            if(analyse_sensor_string(sensor_array, raw_string)){

                /* send sensor data to firebase, return TRUE if sending successed */
                if(Firebase.setString(fbdo, FIREBASE_SENSOR_PATH, raw_string)){

                    lcd_print_sensor_value(sensor_array);
                }
                else{                                                   // push data to firebase failed

                    // Serial.println(fbdo.errorReason());              // Uncomment this line to know reason why push data fail
                    return;
                }
            }
            is_new_data = false;                                        // announce: new data was read
            raw_string.remove(0, raw_string.length());
        }

        /* GET DATA FROM FIREBASE AND SEND IT TO NODE THROUGH LORA********************************/

        // get and send data every 1 seconds (700ms wait + 300ms get data from firebase)
        else if(wait_for(800)){

            /* return TRUE if get data successed, and data will be stored in database(fbdo) */
            if(Firebase.getString(fbdo, FIREBASE_CONTROL_PATH)){

                String control_data = fbdo.stringData();                           // get data from database
                /* analyse control_data, return false if data was not available */
                if((analyse_control_data(control_data.toInt(), &relay_state))){

                    send_byte_lora(relay_state);                        // send new relay state to Node
                    if(Firebase.setString(fbdo, FIREBASE_STATUS_PATH, String(relay_state)));
                    // Serial.println(relay_state);
                    lcd_print_relay_state(relay_state);
                    LoRa.receive();                                     // send completed, renable receive mode
                    EEPROM.write(EEPROM_ADDRESS, relay_state);            // saved relay state
                    EEPROM.commit();
                }
                else{                                                   // data format is not available

                    if(is_door_open){

                        if(wait_for_door(2000)){

                            lcd.setCursor(13, 3);
                            lcd.print("CLOSE");
                            is_door_open = false;
                            if(Firebase.setString(fbdo, FIREBASE_CONTROL_PATH, "50")){

                                return;
                            }
                        }
                    }
                    return;
                }
            }
            else{                                                       // get data failed

                // Serial.println(fbdo.errorReason());                  // Uncomment this line to know reason get data fail
                return;
            }
        }
    }
}
/**********************************************************************************************************************/

/* THIS LOOP RUN ON CORE 0, RECEIVING DATA FROM LORA NODE *************************************************************/
void loop_run_in_core_0(void *parameter){

    /* Put changging receive mode here look better */
    LoRa.onReceive(loRa_on_receive_interrupt);                          // setup interrupt for receiving data from NODE
    LoRa.receive();
    while (true){

        if (is_receive_interrupt){                                      // check if interrupt has occured

            raw_string += LoRa.readString();                            // get data from LoRa buffer
            is_new_data = true;
            if(wait_for(5)){                                            // wait a little bit for not missing any data

                is_receive_interrupt = false;                           // CORE 0 completed receiving process
            }
        }
        vTaskDelay(20);                                                 // relax time for core 0
    }
}
/**********************************************************************************************************************/

/* ANALYSE DATA FUNCTION *******************************************************/

bool analyse_control_data(uint8_t control_byte, uint8_t *state_byte){

    /* control_byte format in decimal: <relay that has change><state you want to change>
        example: 41 mean user want to turn on fourth relay */
    uint8_t tenths = control_byte / 10;
    uint8_t units = control_byte % 10;
    uint8_t temp = *state_byte;
    if(tenths > 5 || tenths < 1 || (units != 0 && units != 1)){

        return false;                                                   // format data is not familiar
    }
    else{

        if(units == 1){

            temp |= (0x01 << (tenths - 1));                      // set relay bit to 1
        }
        else if(units == 0){

            temp &= ~(0x01 << (tenths - 1));                     // clear relay bit
        }
    }
    if(temp != *state_byte){

        *state_byte = temp;
        return true;
    }
    else{

        return false;
    }
}

bool analyse_sensor_string(String *string_array, String raw){

    int i = 0;
    int amount_of_sign = 0;
    while(amount_of_sign < 3){                                          // stop if when found NULL char

        if(raw[i] == '*'){                                              // this star sign was used for divide package

            i++;
            amount_of_sign++;
            continue;
        }
        /* string format: temperature/humidity/air ppm/ -> 3 sign in format package */
        else{

            if(amount_of_sign == 0){                                    // get temperature string
                string_array[0] += raw[i];
            }
            else if(amount_of_sign == 1){                               // get humidity string
                string_array[1] += raw[i];
            }
            else if(amount_of_sign == 2){                               // get gas density string
                string_array[2] += raw[i];
            }
            if(i > 30){                                                 // check if string format was not familiar

                return false;
            }
            i++;
        }
    }
    return true;
}


/* LORA SUB FUNCTION ***********************************************************/
/* recall to this function in case interrupt occur */
void loRa_on_receive_interrupt(int packetLength){

    if(is_receive_interrupt){                                           // check if ESP didn't complete previous receive process

        ESP.restart();
    }
    is_receive_interrupt = true;                                        // set interrupt boolean
}

void init_lora(){                                                       // begin SPI with LoRa

    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
    LoRa.setPins(PIN_SS, PIN_RST, PIN_DI0);
    while (!LoRa.begin(RF_BAND)){

        delay(200);
    }
}

void send_byte_lora(uint8_t package){                                   // send a byte with LoRa

    LoRa.beginPacket();
    LoRa.print(package);
    LoRa.endPacket();
}

/* LCD SUB FUNCTION ************************************************************/

void lcd_setup(){

    lcd.begin();                                                        // setup lcd
    byte water[8] = {

        0x00, 0x04, 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x0E
    };  lcd.createChar(1, water);

    byte temperature[8]= {

        0x03, 0x0B, 0x08, 0x1E, 0x08, 0x08, 0x0A, 0x04
    }; lcd.createChar(2, temperature);


    byte degree[8]= {

        0x18, 0x18, 0x06, 0x09, 0x08, 0x08, 0x09, 0x06
    }; lcd.createChar(3, degree);

    lcd.backlight();
    lcd.setCursor(0, 0);    lcd.print("R1:");                           // print name of relay and sensor
    lcd.setCursor(0, 1);    lcd.print("R2:");
    lcd.setCursor(0, 2);    lcd.print("R3:");
    lcd.setCursor(0, 3);    lcd.print("R4:");
    lcd.setCursor(7, 0);    lcd.write(2);   lcd.setCursor(14, 0);   lcd.write(3);
    lcd.setCursor(7, 1);    lcd.write(1);   lcd.setCursor(14, 1);   lcd.print("%");
    lcd.setCursor(7, 2);    lcd.print("GAS:"); lcd.setCursor(17, 2); lcd.print("ppm");
    lcd.setCursor(7, 3);    lcd.print("DOOR: ");

}

void lcd_print_relay_state(uint8_t relay_byte){

    lcd.setCursor(3, 0);                                                // RELAY 1 state
    if((relay_byte & 0x01) != 0x00){                                    // first LSB of relay_state byte
        lcd.print("ON ");
    }
    else{
        lcd.print("OFF");
    }

    lcd.setCursor(3, 1);                                                // RELAY 2 state
    if((relay_byte & 0x02) != 0x00){                                    // second LSB of relay_state byte
        lcd.print("ON ");
    }
    else{
        lcd.print("OFF");
    }

    lcd.setCursor(3, 2);                                                // RELAY 3 state
    if((relay_byte & 0x04) != 0x00){                                    // third LSB of relay_state byte
        lcd.print("ON ");
    }
    else{
        lcd.print("OFF");
    }

    lcd.setCursor(3, 3);                                                // RELAY 4 state
    if((relay_byte & 0x08) != 0x00){                                    // fourth bit of relay_state byte
        lcd.print("ON ");
    }
    else{
        lcd.print("OFF");
    }
    lcd.setCursor(13, 3);
    if(((relay_byte & 0x10) != 0x00) && !is_door_open){

        lcd.print("OPEN ");
        is_door_open = true;
    }
}

void lcd_print_sensor_value(String *sensor_array){

    lcd.setCursor(9, 0);        lcd.print(sensor_array[0].toFloat());   // temperature value
    lcd.setCursor(9, 1);        lcd.print(sensor_array[1].toFloat());   // humidity value
    lcd.setCursor(11, 2);       lcd.print(sensor_array[2].toFloat());   // air quality value
    lcd.setCursor(17, 2);       lcd.print("ppm");
}

void connect_eps32_wifi(){

    WiFi.begin(WIFI_USERNAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED){

        delay(200);
    }
}

bool wait_for(unsigned long interval){                                  // wait for a time (you can do other work)

    static bool is_enabled_timing = false;
    /* in fact, millis function run over and over, so we need to keep this static to mark your start time */
    static unsigned long previous_millis = 0;
    unsigned long current_millis = millis();
    if(!is_enabled_timing){                                             // check if timer was start to timing before?

        is_enabled_timing = true;
        previous_millis = current_millis;                               // if yes, set previous millis and start couting
    }
    if(current_millis - previous_millis >= interval)                    // check if timing reached limit
    {

        is_enabled_timing = false;                                      // stop using timer
        return true;
    }
    return false;
}

bool wait_for_door(unsigned long interval){                                  // wait for a time (you can do other work)

    static bool is_enabled_timing = false;
    /* in fact, millis function run over and over, so we need to keep this static to mark your start time */
    static unsigned long previous_millis = 0;
    unsigned long current_millis = millis();
    if(!is_enabled_timing){                                             // check if timer was start to timing before?

        is_enabled_timing = true;
        previous_millis = current_millis;                               // if yes, set previous millis and start couting
    }
    if(current_millis - previous_millis >= interval)                    // check if timing reached limit
    {

        is_enabled_timing = false;                                      // stop using timer
        return true;
    }
    return false;
}
