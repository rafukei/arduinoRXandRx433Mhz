# arduinoRXandRx433Mhz
 RC controller and receiver with 2 motors and 4 switches


# Moodin valinta (lähetin):
D2 ---[10k PULLUP]--- +5V
  | (Normaali: HIGH = Lähetin)
  |
 [JUMPER] (Voit poistaa)
  |
 GND (LOW = Vastaanotin)
 
 # lähetin

ARDUINO PRO MINI (LÄHETIN)
==========================
                          _______________________
                         |  ARDUINO PRO MINI     |
                         |                       |
                      +---|VCC           RAW     |--- +5V (USB/virtalähde)
                      |   |GND           GND     |--- GND
                      |   |RST           RST     |
                      |   |RXI           TX0     |--- RX (vain debug)
                      |   |TXO           RX0     |--- TX → VASTAANOTTIMEN RX
                      |   |D2            D2/MODE |--- 
                      |   |D3~           D3/IN2  |--- 
                      |   |D4            D4/IN1  |--- 
                      |   |D5~           D5/ENA  |--- 
                      |   |D6~           D6/ENB  |--- 
                      |   |D7            D7/IN3  |--- 
                      |   |D8            D8/IN4  |--- 
                      |   |A0            A0/BTN_U|--[PUSH]--- GND
                      |   |A1            A1/BTN_D|--[PUSH]--- GND
                      |   |A2            A2/BTN_L|--[PUSH]--- GND
                      |   |A3            A3/BTN_R|--[PUSH]--- GND
                         |_______________________|
## Painonappien kytkentä:

A0 ---[10k PULLUP]--- +5V
     |
    [PUSH BUTTON UP]
     |
    GND


 # Vastaanotin

 ARDUINO PRO MINI (VASTAANOTIN)
===============================
                          _______________________
                         |  ARDUINO PRO MINI     |
                         |                       |
                      +---|VCC           RAW     |--- 7-12V (L298N +5V)
                      |   |GND           GND     |--- GND (jaettu L298N:n kanssa)
                      |   |RST           RST     |
                      |   |RXI           TX0     |--- RX ← LÄHETTIMEN TX
                      |   |TXO           RX0     |--- TX (vain debug)
                      |   |D2            D2/MODE |---[10k]--- GND (MODE_PIN)
                      |   |D3~           D3/IN2  |--- L298N IN2 (Moottori A)
                      |   |D4            D4/IN1  |--- L298N IN1 (Moottori A)
                      |   |D5~           D5/ENA  |--- L298N ENA (PWM A)
                      |   |D6~           D6/ENB  |--- L298N ENB (PWM B)
                      |   |D7            D7/IN3  |--- L298N IN3 (Moottori B)
                      |   |D8            D8/IN4  |--- L298N IN4 (Moottori B)
                      |   |A0            A0      |--- Vapaa
                      |   |A1            A1      |--- Vapaa
                      |   |A2            A2      |--- Vapaa
                      |   |A3            A3      |--- Vapaa
                         |_______________________|

                   
