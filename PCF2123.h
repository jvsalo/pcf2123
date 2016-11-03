/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Jaakko Salo (jaakkos@gmail.com / jaakkos on Freenode)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __PCF2123_H__
#define __PCF2123_H__

/* https://github.com/PaulStoffregen/Time */
#include <Time.h>

/**
 * Structure for holding and manipulating PCF2123 control registers
 * (1 and 2). Use PCF2123::ctrl_get() to get an instance.
 */
struct PCF2123_CtrlRegs
{
  /**
   * PCF2123 registers (1 bit/register). Default values
   * marked with asterisk.
   *
   * Enum values are bit positions into the registers.
   */

  enum Ctrl1Regs {  EXT_TEST  = 7+8,  /* 0*: normal mode, 1: external clock test mode */
                    STOP      = 5+8,  /* 0*: RTC clock runs, 1: RTC clock stopped */
                    SR        = 4+8,  /* 0*: no software reset, 1: initiate software reset */
                    HOUR_MODE = 2+8,  /* 0*: 24h mode, 1: 12h mode. We only use 24h mode */
                    CIE       = 1+8   /* 0*: no correction interrupt generated, 1: pulse
                                         generated at every correction cycle */
  };

  enum Ctrl2Regs {  MI    = 7,  /* 0*: minute int. disabled, 1: minute int. enabled */
                    SI    = 6,  /* 0*: second int. disabled, 1: second int. enabled */
                    MSF   = 5,  /* 0*: no m/s int. generated, 1: int. has been generated */
                    TI_TP = 4,  /* 0*: int. follows timer flags, 1: int. generates pulse */
                    AF    = 3,  /* 0*: no alarm int. generated, 1: alarm triggered */
                    TF    = 2,  /* 0*: no countdown int. generated, 1: countdown triggd */
                    AIE   = 1,  /* 0*: don't generate alarm int., 1: generate alarm int. */
                    TIE   = 0   /* 0*: don't generate countdown int., 1: generate countdown
                                   int. */
  };

  uint16_t ctrl; /**< Control registers 1 (hi byte) and 2 (lo byte) */

  /**
   * Returns value of the given property.
   *
   * @param     bit     Register bit to return
   *
   * @return    Bit value
   */
  bool get(int bit);

  /**
   * Sets the value of the given property.
   *
   * @param     bit     Register bit to modify
   * @param     value   New bit value
   *
   * @return    Old bit value
   */
  bool set(int bit, bool value);

  /**
   * Get/set control register bytes
   */
  uint8_t ctrl1_get();
  uint8_t ctrl2_get();
  void ctrl1_set(uint8_t byte);
  void ctrl2_set(uint8_t byte);

  /**
   * Set all interrupt bits high. This will cause these bits
   * to be ignored if the register is written, because the RTC
   * will internally "AND" the current state with written bits.
   */
  void mask_alarms();
};

/**
 * PCF2123 driver
 */
class PCF2123
{
  private:
    uint8_t       ce_pin_;    /**< Chip select pin */
    SPISettings   spi_cfg_;   /**< SPI configuration */

    /**
     * Do SPI transmit and receive.
     *
     * @param   rw      Read or write (write = 0, read = 1)
     * @param   addr    Register to access
     * @param   buf     Buffer for reading/writing data
     * @param   sz      Number of bytes to transact
     */
    void rxt(uint8_t rw, uint8_t addr, uint8_t *buf, size_t sz);

    /**
     * Parse a BCD-encoded decimal into decimal.
     *
     * @param   bcd     The encoded decimal
     *
     * @return  Decoded decimal
     */
    uint8_t bcd_decode(uint8_t bcd);

    /**
     * Encode a decimal into BCD
     *
     * @param   dec     Decimal to encode
     *
     * @return  Encoded value
     */
    uint8_t bcd_encode(uint8_t dec);

  public:
    /**
     * Construct a new instance of the driver.
     *
     * @param   ce_pin  Chip enable pin for this PCF2123
     */
    PCF2123(uint8_t ce_pin);

    /**
     * Get current time of the RTC.
     *
     * @param   now     Current time is written here
     *
     * @return  True if clock source integrity was
     *          guaranteed
     */
    bool time_get(tmElements_t *now);

    /**
     * Set current time of the RTC.
     *
     * @param   new_time      New time to set
     */
    void time_set(tmElements_t *new_time);

    /**
     * Reset the RTC.
     * NXP recommends doing this after powering on.
     */
    void reset();

    /**
     * Stop/resume the RTC.
     *
     * @param  stopped  Whether the clock should be stopped
     *                  or running
     */
    void clock_stop(bool stopped);

    /**
     * Read control registers.
     *
     * @return  Current register state
     */
    PCF2123_CtrlRegs ctrl_get();

    /**
     * Write control register(s).
     * Either ctrl1, ctrl2 or both can be written in the same
     * transaction.
     *
     * @param   regs        Register buffer
     * @param   set_ctrl1   Write ctrl1 register
     * @param   set_ctrl2   Write ctrl2 register
     * @param   mask_alarms Set alarm bits high to not affect alarm state
     */
    void ctrl_set(const PCF2123_CtrlRegs *regs, bool set_ctrl1, bool set_ctrl2,
                  bool mask_alarms);
};

#endif
