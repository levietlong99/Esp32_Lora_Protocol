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
   -> Read CONFIG.md to know how to setup library, configure it smartly for your project

      Created 8 Janaruary 2021
      by Viet Long
      Contact to my email: vietlongle2000@gmail.com for more information.
*/
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

#define WIFI_USERNAME   "REPLACE YOUR WIFI USER NAME HERE"
#define WIFI_PASSWORD   "REPLACE YOUR WIFI PASSWORD HERE"
#define FIREBASE_HOST   "REPLACE YOUR FIREBASE HOST NAME HERE"
#define FIREBASE_AUTH   "REPLACE YOUR FIREBASE SECRET TOKEN HERE"
/* defines the pins used by the tranceiver module */
#define PIN_SCK         18
#define PIN_MISO        19
#define PIN_MOSI        23
#define PIN_SS          5
#define PIN_RST         14
#define PIN_DI0         26                      // in my experience, this's the best interrupt pin on ESP32

#define firebase_sensor_path     "/sensor_string"
#define firebase_controller_path "/controller_byte"

#define BAND            433E6                   // Asia - Viet Nam

void connect_eps32_wifi();
void init_lora();
void send_byte_lora(uint8_t package);
//void get_temperature_and_humidity(char *temperature, char *humidity, char *raw);
bool wait_for(unsigned long interval);
void loRa_on_receive_interrupt(int packetLength);

/* Define FirebaseESP8266 data object for data sending and receiving */
FirebaseData fbdo;

volatile bool is_new_data = false;
volatile bool is_receive_interrupt{false};
uint8_t controller_byte = 0;                    // use 4 LSB of byte to control device
String raw_string = "";

void setup() {

    delay(1000);                                // wait for ESP32 start
    //Serial.begin(115200);                     //debug, we can see hardware change at this baud
    connect_eps32_wifi();
    init_lora();
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);// connect esp32 to firebase
    /* Uncomment below line to enable syncworld - avoid signal from other LoRa*/
    // LoRa.setSyncWord(0xF3);

    /* Enable auto reconnect the WiFi when connection lost */
    Firebase.reconnectWiFi(true);
    /* Syntax to creat loop on another core , run paralell with main loop */
    xTaskCreatePinnedToCore(

        loop_run_in_core_0,                     // loop function run in core 0
        "loop_run_in_core_0",                   // name of the task (for debugging)
        5000,                                   // stack size for core 0 (bytes)
        NULL,                                   // paramester to pass in loop
        1,                                      // task priority (set this to one so we dont miss any data)
        NULL,                                   // task handle (just run loop 0)
        0                                       // core that the task run on
    );
}

/* This loop run on core 1 (connect with Firebase, sending with LoRa) */
void loop() {

    /* wait for CPU complete it's task, avoid hardware watchdog */
    yield();
    if(is_receive_interrupt) {

        return;                                 // wait for core 0 complete
    }
    else{

        if(is_receive_interrupt){

            return;                             // recheck at this point  seem better
        }
        if(is_new_data) {                       // new interrupt ended, new data need to receive

            is_new_data = false;
            /* send sensor data to firebase, if  setString func dont success, return 0 */
            if(Firebase.setString(fbdo, firebase_sensor_path, raw_string)){

                // Serial.println(raw_string);  // follow flow
                /* clear sensor string */
                raw_string.remove(0, raw_string.length());
            }
            else{

                /* get error  information for debugging */
                // Serial.println(fbdo.errorReason());
                ESP.restart();                  // quick restart
            }
        }
        // send data after ~ 1000ms (getInt function has delay time ~ 350ms)
        else if(wait_for(650)){

            if(is_receive_interrupt){

                return;                         // recheck at this point  seem better
            }
            /* get data from firebase, if getInt func dont success, return  0 */
            if(Firebase.getInt(fbdo, firebase_controller_path)){

                /* if getInt success, get controller byte from saved database */
                controller_byte = fbdo.intData();
                // Serial.println(controller_byte);// follow flow
                if (controller_byte > 15) {     // control 4 devices, max value = 0b1111 = 15

                  return;
                }
                /* forward received information to slave */
                send_byte_lora(controller_byte);
                /* Back to receive mode (required for the next interrupt) */
                LoRa.receive();
            }
            else{

                /* get error  information for debugging */
                // Serial.println(fbdo.errorReason());
                ESP.restart();                  // quick restart
            }
        }
    }
}

/* we need to put interrupt short as possible,
   if we put LoRa receive script inside interrupt, ESP32 may crash */
void loRa_on_receive_interrupt(int packetLength){

    is_receive_interrupt = true;                // set interrupt flag
}

/* This loop run on core 0 (Receving data from slave with LoRa) */
void loop_run_in_core_0(void *parameter){

    /* Put changging receive mode here look better */
    LoRa.onReceive(loRa_on_receive_interrupt);  // callback to loRa_on_receive_interrupt when interrupt occur
    LoRa.receive();
    while (true){                               // core 0 always in this loop

        if (is_receive_interrupt){

            raw_string += LoRa.readString();    // get data in LoRa buffer(store in STACK)
            is_new_data = true;
            if(wait_for(5)){                    // time need to complete receive (stop main loop)

                is_receive_interrupt = false;
            }
        }
        vTaskDelay(20);                         // wait before next action (delay in core 0)
    }
}

void connect_eps32_wifi(){

    WiFi.begin(WIFI_USERNAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED){

        delay(200);
    }
}

void init_lora(){                               // begin SPI with LoRa

    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
    LoRa.setPins(PIN_SS, PIN_RST, PIN_DI0);
    while (!LoRa.begin(BAND)){

        delay(200);
    }
}

void send_byte_lora(uint8_t package){           // send a byte with LoRa

    LoRa.beginPacket();
    LoRa.print(package);
    LoRa.endPacket();
}

bool wait_for(unsigned long interval){          // wait for a time (using timer)

    static bool is_first_time = true;
    static unsigned long previous_millis = 0;
    unsigned long current_millis = millis();
    if(is_first_time){
        is_first_time = false;
        /* in case of running for the first time, set previous millis */
        previous_millis = current_millis;
    }
    if(current_millis - previous_millis >= interval)
    {
        /* reset first time identify bool for next usage */
        is_first_time = true;
        return true;
    }
    return false;
}
