#ifndef PCA9685_H
#define PCA9685_H

// REGISTER ADDRESSES
#define PCA9685_MODE1_REG     0x00  /**< Mode Register 1 */
#define PCA9685_PRESCALE_REG  0xFE  /**< Prescaler for PWM output frequency */
#define PCA9685_LED0_ON_L_REG 0x06  /**< LED0 on tick, low byte*/

// MODE1 bits
#define MODE1_RESTART 0x80  /**< Restart enabled */
#define MODE1_SLEEP   0x10  /**< Low power mode. Oscillator off */
#define MODE1_AI      0x20  /**< Auto-Increment enabled */

//first byte in any transmission has format: [start_bit, addr(7bits), mode_bite(w=0, r=1), slave_ack_bit]
//real address of device is 0x41, but there are a LSB mode bit, so 0x82 is first byte to communicate with 0x42 device.
//to use read mode just use PCA9685_ADDR | 1
#define PCA9685_ADDR  0x82
#endif