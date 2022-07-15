#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "picoUART/picoUART.h"
#include "PCA9685.h"

#define I2C_SDA         PB3                   // i2c data pin
#define I2C_SCL         PB4                   // i2c clock pin

// -----------------------------------------------------------------------------
// I2C Master Implementation (Write only)
// -----------------------------------------------------------------------------

// I2C macros
#define I2C_SDA_HIGH()  DDRB &= ~(1<<I2C_SDA) // release SDA   -> pulled HIGH by resistor
#define I2C_SDA_LOW()   DDRB |=  (1<<I2C_SDA) // SDA as output -> pulled LOW  by MCU
#define I2C_SCL_HIGH()  DDRB &= ~(1<<I2C_SCL) // release SCL   -> pulled HIGH by resistor
#define I2C_SCL_LOW()   DDRB |=  (1<<I2C_SCL) // SCL as output -> pulled LOW  by MCU

// I2C init function
void I2C_init() {
  DDRB  &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // pins as input (HIGH-Z) -> lines released
  PORTB &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // should be LOW when as output
}

// I2C transmit one data byte to the slave, ignore ACK bit, no clock stretching allowed
void I2C_write(uint8_t data)
{
  for(uint8_t i = 8; i; i--) {            // transmit 8 bits, MSB first
    I2C_SDA_LOW();                        // SDA LOW for now (saves some flash this way)
    if (data & 0x80) I2C_SDA_HIGH();      // SDA HIGH if bit is 1
    I2C_SCL_HIGH();                       // clock HIGH -> slave reads the bit
    data<<=1;                             // shift left data byte, acts also as a delay
    I2C_SCL_LOW();                        // clock LOW again
  }
  I2C_SDA_HIGH();                         // release SDA for ACK bit of slave
  I2C_SCL_HIGH();                         // 9th clock pulse is for nine bit
  asm("nop");                             // ACK bit is ignored if exists, just a delay
  I2C_SCL_LOW();                          // clock LOW again
}

uint8_t I2C_read()
{
  uint8_t output = 0x00;
  I2C_SDA_HIGH();
  for(uint8_t i = 8; i; i--)
  {
    I2C_SCL_HIGH();
    if(PINB & (1<<I2C_SDA)) {output = output | (1<<(i-1));}
    I2C_SCL_LOW();
  }
  //send NACK
  I2C_SDA_HIGH();
  I2C_SCL_LOW();
  return output;
}

// I2C start transmission
void I2C_start(uint8_t addr)
{
  I2C_SDA_LOW();                          // start condition: SDA goes LOW first
  I2C_SCL_LOW();                          // start condition: SCL goes LOW second
  I2C_write(addr);                        // send slave address
}

// I2C stop transmission
void I2C_stop() {
  I2C_SDA_LOW();                          // prepare SDA for LOW to HIGH transition
  I2C_SCL_HIGH();                         // stop condition: SCL goes HIGH first
  I2C_SDA_HIGH();                         // stop condition: SDA goes HIGH second
}

//I2C restart transmission; used for read operation
void I2C_restart(uint8_t addr)
{
  I2C_SDA_HIGH();
  I2C_SCL_HIGH();
  addr = addr | 1; //setting last byte to 1; used to set read mode
  I2C_start(addr);
}

// -----------------------------------------------------------------------------
// PCA9685 Implementation
// -----------------------------------------------------------------------------
void PCA9685_write8(uint8_t reg, uint8_t data)
{
  I2C_start(PCA9685_ADDR);
  I2C_write(reg);
  I2C_write(data);
  I2C_stop();
}

uint8_t PCA9685_read8(uint8_t reg)
{
  I2C_start(PCA9685_ADDR);
  I2C_write(reg);
  I2C_restart(PCA9685_ADDR);
  uint8_t data = I2C_read();
  I2C_stop();
  return data;
}

void PCA9685_reset()
{
  PCA9685_write8(PCA9685_MODE1_REG, MODE1_RESTART);
  _delay_ms(10);
}

void PCA9685_begin()
{
  I2C_init();
  PCA9685_reset();
  //initialize prescaler to max freq
  PCA9685_write8(PCA9685_MODE1_REG, MODE1_SLEEP);
  PCA9685_write8(PCA9685_PRESCALE_REG, 3);
  asm("nop");
  asm("nop");
  asm("nop");
  PCA9685_write8(PCA9685_MODE1_REG, MODE1_AI);
  asm("nop");
  asm("nop");
  asm("nop");
}

void PCA9685_setPWM(uint8_t pin, uint16_t value)
{
  I2C_start(PCA9685_ADDR);                     //initialize for writing
  I2C_write(PCA9685_LED0_ON_L_REG + 4*pin);
  I2C_write(0x00);
  I2C_write(0x00);
  I2C_write(value);
  I2C_write(value >> 8);
  I2C_stop();
}

uint16_t PCA9685_getPWM(uint8_t pin)
{
  uint8_t pwm_LSB = PCA9685_read8(PCA9685_LED0_ON_L_REG + 4*pin + 2);
  uint8_t pwm_MSB = PCA9685_read8(PCA9685_LED0_ON_L_REG + 4*pin + 3);
  uint16_t value = (pwm_MSB << 8) | pwm_LSB;
  return value;
}

// -----------------------------------------------------------------------------
// EEPROM operarions
// -----------------------------------------------------------------------------
uint16_t get_ch_value(uint8_t ch_num, uint8_t mode) //mode=1 enables offset of 16 words from beggining
{
  uint16_t addr = ch_num*2;
  if(mode == 1) {addr += 32;}
  return eeprom_read_word((uint16_t*)addr);
}

void set_ch_value(uint8_t ch_num, uint8_t mode, uint16_t value) //mode=1 enables offset of 16 words from beggining
{
  uint16_t addr = ch_num*2;
  if(mode == 1) {addr += 32;}
  eeprom_update_word((uint16_t*)addr, value);
}


// -----------------------------------------------------------------------------
// Main block
// -----------------------------------------------------------------------------

//current working mode
//0 - off
//1 - EEPROM 0
//2 - EEPROM 1
uint8_t current_mode = 0;

void switch_mode(uint8_t mode)
{
  current_mode = mode;
  for(int8_t ch = 16; ch >= 0; ch--)
  {
    if(mode == 0) {PCA9685_setPWM(ch, 0);}
    else          {PCA9685_setPWM(ch, get_ch_value(ch, mode-1));}
  }
}

//button ISR
ISR(INT0_vect)
{
  current_mode++;
  if(current_mode == 3) {current_mode = 0;}
  switch_mode(current_mode);
}

int main()
{
  //configure button
  DDRB &= ~(1<<PB0);
  PORTB |= (1<<PB0);
  GIMSK |= (1<<INT0);
  //configure RX
  DDRB  &= ~(1<<PB1);
  PORTB |= (1<<PB1);

  PCA9685_begin();
  switch_mode(0);

  sei();
  
  while(true)
  {
    uint8_t cmd = purx();
    asm("nop");
    //delay between bytes must be at least 5 ms
    //available commands:
    //0x01 - set channel value (next bytes: channel, value_MSB, value_LSB); range: 0-4095
    //0x02 - save channel value to EEPROM (next bytes: channel, value_MSB, value_LSB, mode)
    //0x03 - change current working mode (next byte - mode; values: 0x00 - all off, 0x01 - mode 0, 0x02 - mode 1)
    //       this command also can be used for returning all channels to stored values
    //0x04 - read saved values for mode (next bytes: mode)
    //       answer is 32 bytes of data in format: [MSB0, LSB0], [MSB1, MSB1], ..., [MSB15, LSB15]
    //       there are 16 words representing values of 16 channels
    //0x05 - read current values of all channels
    //       answer is 32 bytes of data in format: [MSB0, LSB0], [MSB1, MSB1], ..., [MSB15, LSB15]
    //       there are 16 words representing values of 16 channels
    if(cmd == 0x01)
    {
      uint8_t channel = purx();
      asm("nop");
      uint8_t MSB = purx();
      asm("nop");
      uint8_t LSB = purx();
      uint16_t value = (MSB << 8) | LSB;
      PCA9685_setPWM(channel, value);
    }
    if(cmd == 0x02)
    {
      uint8_t channel = purx();
      asm("nop");
      uint8_t MSB = purx();
      asm("nop");
      uint8_t LSB = purx();
      asm("nop");
      uint8_t mode = purx();
      uint16_t br = (MSB << 8) | LSB;
      set_ch_value(channel, mode, br);
    }
    if(cmd == 0x03)
    {
      uint8_t mode = purx();
      switch_mode(mode);
    }
    if(cmd == 0x04)
    {
      uint8_t mode = purx();
      for(uint8_t ch = 0; ch < 16; ch++)
      {
        uint16_t ch_val = get_ch_value(ch, mode);
        uint8_t LSB = (uint8_t)ch_val;
        uint8_t MSB = (uint8_t)(ch_val >> 8);
        putx(MSB);
        putx(LSB);
      }
    }
    if(cmd == 0x05)
    {
      for(uint8_t ch = 0; ch < 16; ch++)
      {
        uint16_t ch_val = PCA9685_getPWM(ch);
        uint8_t LSB = (uint8_t)ch_val;
        uint8_t MSB = (uint8_t)(ch_val >> 8);
        putx(MSB);
        putx(LSB);
      }
    }
  }
}