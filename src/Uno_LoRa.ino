/*
    #PROJECT: Using esp32 to communicate with firebase, communicate with Arduino Uno through RF (LoRa module).

    Arduino side: get control value from esp 32 and control relays, reading sensor and send back to ESP32.

    #OUTSTANDING FEATURE!
    - Using timer interrupt on atmega328, so missing data is reduced at least!!
    - Good arrange process for transmit and receive, so process is very sepreate.
    - High speed response, 1 seconds and relay under your controller.
    - Read 7 invidual sensor values at a time, and sends back stability!
    - Add library with smart method, disable what make your process slower.
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
#include <DHT.h>
#include <MQ2.h>

#define BAND        433E6                                          // Asia - Viet Nam
#define DEVICE_1    3                                              // digital pin for controlling delay
#define DEVICE_2    4
#define DEVICE_3    5
#define DEVICE_4    6

#define FIRST_BIT   0x01
#define SECOND_BIT  0x02
#define THIRD_BIT   0x04
#define FOURTH_BIT  0x08

#define SMOKE_PIN   A2                                              // using smoke sensor (MQ2)
#define DHT11_PIN   A0                                              // using temperature and humidity sensor
#define LIGHT_PIN   A1                                              // using light sensor (Mh from Flying fish)

void init_lora();
void send_sensor_data_lora();
bool wait_for(unsigned long interval);
void setup_timer_interrupt_4s();
void reset_board();

/* sensor value */
float temperature, humidity;
uint8_t brightness;
float lpg, co, smoke;                                               // type of gas LPG, CO & SMOKE

/* setup library */
DHT dht(DHT11_PIN, DHT11);                                          // dht(DHTPIN, DHTTYPE)
MQ2 mq2(SMOKE_PIN);

/* process variable */
uint8_t data_receive;
volatile bool  is_time_send = false;
String string_receive = "";
String string_send = "";
volatile  static long long int non_send_times = 0;                  // for bad circumstance

void setup() {

    // Serial.begin(115200);                                        // debug
    init_lora();
    dht.begin();                                                    // init dht11
    mq2.begin();                                                    // init mq2 sensor
    pinMode(DEVICE_1, OUTPUT);
    pinMode(DEVICE_2, OUTPUT);
    pinMode(DEVICE_3, OUTPUT);
    pinMode(DEVICE_4, OUTPUT);
    pinMode(SMOKE_PIN, INPUT);
    pinMode(LIGHT_PIN, INPUT);
    pinMode(DHT11_PIN, INPUT);
    setup_timer1_interrupt_4s();
}

void loop() {

    yield();                                                        // wait for CPU complete it's work
    // try to parse packet
    if (LoRa.parsePacket() != 0) {

        while (LoRa.available() || !wait_for(10)){                  // wait_for 10ms to make sure receiving complete

            string_receive += LoRa.readString();                    // LoRa read return int type
        }
        /* control 4 relays , 4 LSB of receive byte is relay state */
        data_receive = string_receive.toInt();                      // String type to Int type
        string_receive.remove(0, string_receive.length());
        digitalWrite(DEVICE_1, data_receive & FIRST_BIT);
        digitalWrite(DEVICE_2, data_receive & SECOND_BIT);
        digitalWrite(DEVICE_3, data_receive & THIRD_BIT);
        digitalWrite(DEVICE_4, data_receive & FOURTH_BIT);
        // Serial.println(data_receive);                            // debug
        while(!wait_for(10)){                                       // wait within 100ms from receive complete

            if(is_time_send){                                       // time to send data

                read_sensor();
                send_sensor_data_lora();
                LoRa.receive();
                is_time_send = false;
            }
        }
    }
}

/* remember to keep interrupt so short as possible */
ISR(TIMER1_COMPA_vect){                                             // interrupt function for timer1

    if(non_send_times > 3){

        reset_board();
    }
    else if(is_time_send == true{

        non_send_times++;
    }
    else{

        non_send_times = 0;
        is_time_send = true;                                        // every 4s, got this TRUE
    }
}

void init_lora(){

    if (!LoRa.begin(BAND)){
        while (1);
    }
}

void send_sensor_data_lora(){

    /* add sensor value to string */
    string_send += temperature;
    string_send += "/";
    string_send += humidity;
    string_send += "/";
    string_send += brightness;
    string_send += "/";
    string_send += lpg;
    string_send += "/";
    string_send += co;
    string_send += "/";
    string_send += smoke;
    // Serial.println(string_send);                                     // debug
    LoRa.beginPacket();
    LoRa.print(string_send);                                            // LoRa.print(String_type)
    LoRa.endPacket();
    string_send.remove(0, string_send.length());                        // clear string
}

bool wait_for(unsigned long interval){                                  // wait for a time (using timer)

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

void setup_timer1_interrupt_4s(){

    TCCR1A = 0;                                                         // set entire TCCR1A register to 0
    TCCR1B = 0;                                                         // same for TCCR1B
    TCNT1  = 0;                                                         //initialize counter value to 0
    // set compare match register for 0.25 hz increments
    OCR1A = 62496;                                                      // = (16*4*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei();                                                              // allow interrupt
}

void read_sensor(){

    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    brightness = analogRead(LIGHT_PIN);                                 // read brightness (from 0 - 1023)
    lpg = mq2.readLPG();                                                // read 3 type of gas
    co = mq2.readCO();
    smoke = mq2.readSmoke();
}

void reset_board(){

    asm volatile("jmp 0");                                              // assembly code: jump to line 0 - reset
}
