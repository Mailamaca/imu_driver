
// ref: https://cdn.shopify.com/s/files/1/1509/1638/files/ESP_-_32_NodeMCU_Developmentboard_Pinout_Diagram.png?7487926392378435209

// ref: https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/driver/driver/i2c.h
// ref: https://github.com/juri117/esp32-i2c-slave/blob/master/main/i2c_example_main.cpp

// ref: https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/driver/driver/pcnt.h
// ref: https://nodemcu.readthedocs.io/en/dev-esp32/modules/pulsecnt/#pulsecntobjsetfilter
// ref: https://github.com/DevX8000/ESP32-PCNT-Arduino-Example
// ref: https://github.com/DevX8000/ESP32-PCNT-Arduino-Example/blob/master/src/main.cpp


#include "driver/gpio.h"
#include "maila_encoder.h"
#include "maila_i2c_slave.h"


MailaEncoder *encoderMotor;
MailaI2CSlave *i2c;

void setup() {
  // put your setup code here, to run once:

  encoderMotor = new MailaEncoder(23,-1,PCNT_CHANNEL_0, PCNT_UNIT_0, 30000, 0, 1);

  i2c = new MailaI2CSlave(I2C_NUM_0, 21, 22, 0x09, 1000);


}

void loop() {
  // put your main code here, to run repeatedly:
  
  printf("e1 delta: %06d value: %06d\n", encoderMotor->getDelta(), encoderMotor->getValue());
  
  if (i2c->check_for_data()) {
    uint16_t value = 32000;
    uint8_t replBuff[2];
    replBuff[0] = (uint8_t)(value >> 0);
    replBuff[1] = (uint8_t)(value >> 8);
    i2c->writeData(replBuff, 2);   
    printf("SENT\n");   
  }
  //delay(1000);
}
