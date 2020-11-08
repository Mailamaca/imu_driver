
#include "driver/i2c.h"

#define I2C_SLAVE_TX_BUF_LEN 256  //(2 * DATA_LENGTH)
#define I2C_SLAVE_RX_BUF_LEN 256  //(2 * DATA_LENGTH)

#define I2C_ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define I2C_ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define I2C_ACK_VAL 0x0       /*!< I2C ack value */
#define I2C_NACK_VAL 0x1      /*!< I2C nack value */

#define SLAVE_REQUEST_WAIT_MS 100

class MailaI2CSlave {
  private:
  
    uint8_t outBuff[I2C_SLAVE_TX_BUF_LEN];
    uint16_t outBuffLen = 0;
    
    uint8_t inBuff[I2C_SLAVE_RX_BUF_LEN];
    uint16_t inBuffLen = 0;

    i2c_port_t port;
    gpio_num_t pinSDA;
    gpio_num_t pinSCL;
    int address;
    int readTimeout;
        

  public:
    MailaI2CSlave(i2c_port_t port,
                  int pinSDA,
                  int pinSCL,
                  int address,
                  int readTimeout) {
      this->port = port;
      this->pinSDA = (gpio_num_t)pinSDA;
      this->pinSCL = (gpio_num_t)pinSCL;
      this->address = address;
      this->readTimeout = readTimeout;

      i2c_config_t conf_slave;
      conf_slave.sda_io_num = this->pinSDA;
      conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
      conf_slave.scl_io_num = this->pinSCL;
      conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
      conf_slave.mode = I2C_MODE_SLAVE;
      conf_slave.slave.addr_10bit_en = 0;
      conf_slave.slave.slave_addr = this->address;
      i2c_param_config(this->port, &conf_slave);

      i2c_driver_install(this->port,
                         conf_slave.mode,
                         I2C_SLAVE_RX_BUF_LEN,
                         I2C_SLAVE_TX_BUF_LEN,
                         0);                 
                  
    }

    bool check_for_data() {
      int bytesRead = i2c_slave_read_buffer(this->port,this->inBuff,1,this->readTimeout);
      i2c_reset_rx_fifo(this->port);
      return bytesRead > 0;
    }

    bool writeData(uint8_t *buff, int buffLength) {
      i2c_reset_tx_fifo(this->port);
      i2c_slave_write_buffer(this->port,buff,buffLength,this->readTimeout);
    }

    
};
