# Esp32_Lora_Protocol
Using Esp32 to connect to firebase and using Lora protocol  

![LoRa-ESP32-thumbnail-1](https://user-images.githubusercontent.com/75990061/104584346-0a6d8e80-5695-11eb-8f85-0be6289d881f.jpg)

## About this project  
* Do interesting things with NODE-MCU board is awesome, right? Control every thing in your house with an app connect with WIFI, and you got ***Smart Home***.  
* But ... when you need to control something outside your house, this will be hard. WIFI is a very high band wave that can bring so much data, so you can watch video, download games,..But because of it, WIFI cannot go far. Something need to be control and far from you a little bit, like farm, garden, or may be a light bulb outside in mid night for  fear of ghosts people ....
* My plan is using low band wave - ***RF***, opposite to WIFI, it's speed is just go around of 10 - 400kbps. But trust me, this can bring data with very long distance.  
* In my project, I use ***ESP 32*** that has two core run parallel. May be you know that ESP's "traditional" is ***Hard ware watchdog reset***. The reason is when ESP connect with WIFI, it use up very much of RAM CPU, so if you have been in loop for a long time and don't give it time, they Crash. Beside of that, board like ESP8266 has only 1 core ***(core 0)***. So crash will be easier to  happened. In my code, you will see how i keep balance for CPU without any unpredictable crash.

## What you need to prepare hardware

* In my case, I use NODE MCU 32S ***(38 pin version)*** for controlling gateway.
* Arduino Uno  for controlling NODE.
* LoRa Sx1278 module, Regular and basic LoRa module .  
* DHT11 sensor, which give you temperature and humidity.
* Using MQ2 for analyze LPG, CO, SMOKE density.
* 4 RELAY to control device (***Remember*** to supply different voltage for this from board power source or noise will make you crazy).
* 1 LCD I2C module and LCD 2004 for display relay state and sensor values (***This will be used for gateway of course, going outside to see information display is not smart move!***).

> For more specific information in my code, you can see my GUIDE.md. In that file, I will complain my code clearly and give you some refer links to research.

> If possible, I will upload running video and guide video in the future. So be patient.

### For more information, please contact with my email:
> vietlongle2000@gmail.com  
>

### See you later!
