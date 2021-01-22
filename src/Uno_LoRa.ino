/*
    #PROJECT: Using esp32 to communicate with firebase, communicate with Arduino Uno through RF (LoRa module).

    Arduino side: get control value from esp 32 and control relays, reading sensor and send back to ESP32.

    #OUTSTANDING FEATURE!
    - Using timer interrupt on atmega328, so missing data is reduced at least!!
    - Good arrange process for transmit and receive, so process is very sepreate.
    - High speed response, 1 seconds and relay under your controller.
    - Safe? but we still prepared in case bad circumstance happended.

    #NOTICE
    -> Read README.md for more information about project like: circuit, refer sites, ability,...

    Created 8 Janaruary 2021
    by Viet Long
    Contact to my email: vietlongle2000@gmail.com for more information.
*/
#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <MQUnifiedsensor.h>


#define BAND        433E6                                               // Asia - Viet Nam
#define DEVICE_1    5                                                   // digital pin for controlling delay
#define DEVICE_2    3
#define DEVICE_3    4
#define DEVICE_4    8

#define FIRST_BIT   0x01
#define SECOND_BIT  0x02
#define THIRD_BIT   0x04
#define FOURTH_BIT  0x08

#define GAS_PIN     A1                                                  // using gas sensor (MQ135)
#define DHT11_PIN   A0                                                  // using temperature and humidity sensor

void init_lora();
void read_and_send_sensor_data_lora();
bool wait_for(unsigned long interval);
void setup_timer_interrupt_4s();
float calibrate_NH4_density();
void init_gas_sensor();

void (* reset_board)(void) = 0;


/* setup library */
DHT dht(DHT11_PIN, DHT11);                                              // dht(DHTPIN, DHTTYPE)
// MQUnifiedsensor ...(board name, Voltage_Resolution, ADC_Bit_Resolution, pin, type)
MQUnifiedsensor MQ135("Arduino UNO", 5, 10, GAS_PIN, "MQ-135");         // init MQ135


/* process variable */
volatile bool  is_time_send = false;

void setup() {

     // Serial.begin(115200);                                           // debug
     pinMode(DEVICE_1, OUTPUT);
     pinMode(DEVICE_2, OUTPUT);
     pinMode(DEVICE_3, OUTPUT);
     pinMode(DEVICE_4, OUTPUT);
     pinMode(GAS_PIN, INPUT);
     pinMode(DHT11_PIN, INPUT);
     digitalWrite(DEVICE_1, HIGH);                                      // Using relay trigger low (1 - LOW, 0 - HIGH)
     digitalWrite(DEVICE_2, HIGH);
     digitalWrite(DEVICE_3, HIGH);
     digitalWrite(DEVICE_4, HIGH);

     dht.begin();                                                       // init dht11
     init_gas_sensor();
     delay(1000);
     setup_timer1_interrupt_4s();
     init_lora();
}

void loop() {

    String string_receive = "";
    // try to parse packet
    if (LoRa.parsePacket() != 0) {

        while(LoRa.available()){

                string_receive += LoRa.readString();                    // LoRa read return int type
        }
        /* control 4 relays , 4 LSB of receive byte is relay state */
        uint8_t data_receive = string_receive.toInt();                  // String type to Int type
        data_receive = ~data_receive & 0x0F;                            // flipping bit and clear 4 MSB (Using relay trigger low)
        digitalWrite(DEVICE_1, data_receive & FIRST_BIT);
        digitalWrite(DEVICE_2, data_receive & SECOND_BIT);
        digitalWrite(DEVICE_3, data_receive & THIRD_BIT);
        digitalWrite(DEVICE_4, data_receive & FOURTH_BIT);
         // Serial.println(string_receive);
        while(is_time_send && !wait_for(100)){                          // time to send data (only send within 10ms)

            LoRa.idle();                                                // put LoRa to concentrate mode (important)
            read_and_send_sensor_data_lora();                           // send sensor data
            is_time_send = false;
            delay(100);                                                 // very important time for change mode
            LoRa.receive();
        }
    }
}

/* remember to keep interrupt so short as possible */
ISR(TIMER1_COMPA_vect){                                                 // interrupt function for timer1

    volatile  static uint8_t non_send_times = 0;                        // for bad circumstance
    if(non_send_times > 10){

        // Serial.println("RESET");
        reset_board();
    }
    else if(is_time_send == true){

        non_send_times++;
    }
    else{

        non_send_times = 0;
        is_time_send = true;                                            // every 4s, got this TRUE
    }
}

void init_lora(){

    if (!LoRa.begin(BAND)){
        wait_for(1000)
    }
}

void read_and_send_sensor_data_lora(){

    String string_send = "";
    /* add sensor value to string */
    string_send += dht.readTemperature();
    string_send += "*";
    string_send += dht.readHumidity();
    string_send += "*";
    string_send += calibrate_NH4_density();
    string_send += "*";
    // Serial.println(string_send);                                     // debug
    LoRa.beginPacket();
    LoRa.print(string_send);                                            // LoRa.print(String_type)
    LoRa.endPacket();
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

    /* fix register on atmega328a -pu */
    TCCR1A = 0;                                                         // set entire TCCR1A register to 0
    TCCR1B = 0;                                                         // same for TCCR1B
    TCNT1  = 0;                                                         //initialize counter value to 0
    // set compare match register for 0.25 hz increments
    OCR1A = 65535;                                                      // = (16*4*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei();                                                              // allow interrupt
}

void init_gas_sensor(){

    //Set math model to calculate the PPM concentration and the value of constants
    MQ135.setRegressionMethod(1);
    MQ135.init();
    /*****************************  MQ CAlibration ********************************************/
    // Explanation:
    // In this routine the sensor will measure the resistance of the sensor supposing before was pre-heated
    // and now is on clean air (Calibration conditions), and it will setup R0 value.
    // We recomend execute this routine only on setup or on the laboratory and save on the eeprom of your arduino
    // This routine not need to execute to every restart, you can load your R0 if you know the value
    float calcR0 = 0;
    for(int i = 1; i <= 10; i++){

          MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
          calcR0 += MQ135.calibrate(3.6);
    }
    MQ135.setR0(calcR0/10);
    /*
            Exponential regression:
            GAS      | a      | b
            CO       | 605.18 | -3.937
            Alcohol  | 77.255 | -3.18
            CO2      | 110.47 | -2.862
            Tolueno  | 44.947 | -3.445
            NH4      | 102.2  | -2.473
            Acetona  | 34.668 | -3.369
    */
    MQ135.setA(102.2);                                                  // Calibrate NH4 density
    MQ135.setB(-2.473);
}

float calibrate_NH4_density(){

    MQ135.update();                                                     // read data from SMOKE_PIN
    return MQ135.readSensor();
}
