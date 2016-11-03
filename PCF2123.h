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
#include <SPI.h>

/**
 * Structure for holding and manipulating PCF2123 control registers
 * (1 and 2). Use PCF2123::ctrl_get() to get current state.
 */
struct PCF2123_CtrlRegs
{
  /**
   * Control register bits. Default values marked with asterisk.
   * Enum values are bit positions into the registers.
   */

  enum Ctrl1Regs {  EXT_TEST  = 7+8,  /* 0*: normal mode, 1: external clock test mode */
                    STOP      = 5+8,  /* 0*: RTC clock runs, 1: RTC clock stopped */
                    SR        = 4+8,  /* 0*: no software reset, 1: initiate software reset */
                    HOUR_MODE = 2+8,  /* 0*: 24h mode, 1: 12h mode. We only use 24h mode */
                    CIE       = 1+8   /* 0*: no correction interrupt generated, 1: pulse
                                         generated at every correction cycle */
  };

  enum Ctrl2Regs {  MI    = 7,  /* 0*: minute int. disabled, 1: minute int. enabled */
                    SI    = 6,  /* 0*: second int. disabled, 1: second int. enabled */
                    MSF   = 5,  /* 0*: no m/s int. generated, 1: int. has been generated */
                    TI_TP = 4,  /* 0*: int. follows timer flags, 1: int. generates pulse */
                    AF    = 3,  /* 0*: no alarm int. generated, 1: alarm triggered */
                    TF    = 2,  /* 0*: no countdown int. generated, 1: countdown triggd */
                    AIE   = 1,  /* 0*: don't generate alarm int., 1: generate alarm int. */
                    TIE   = 0   /* 0*: don't generate countdown int., 1: generate countdown
                                   int. */
  };

  uint8_t ctrl[2]; /**< Control registers 1 and 2 */

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
     * Fill in buf[0] with the CMD_BYTE macro before
     * calling.
     *
     * @param   buf     Buffer for reading/writing data
     * @param   sz      Number of bytes to transact
     */
    void rxt(uint8_t *buf, size_t sz);

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
    enum CountdownSrcClock { CNTDOWN_CLOCK_4096HZ   = 0,
                             CNTDOWN_CLOCK_64HZ     = 1,
                             CNTDOWN_CLOCK_1HZ      = 2,
                             CNTDOWN_CLOCK_1PER60HZ = 3 };
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
    void stop(bool stopped);

    /**
     * Set the output frequency of the CLKOUT pin or disable
     * it.
     *
     * Valid frequencies (in Hz) are: 0 (drive CLKOUT to
     * high-Z), 1, 10, 1024, 2048, 4096, 8192, 16384, 32768.
     *
     * @param   freq    CLKOUT square wave frequency
     *
     * @return  True if the given frequency was valid
     */
    bool clkout_freq_set(uint16_t freq);

    /**
     * Configure the countdown timer.
     * When the timer expires, the control register will
     * be modified and an interrupt is generated, if enabled.
     *
     * The countdown period becomes: value / f, where f is
     * the frequency selected by the source_clock parameter.
     *
     * Setting value to 0 stops the timer.
     *
     * Calling this function will reset the countdown timer,
     * so the next interrupt after the call will occur after
     * the configured period (1/f).
     *
     * @param   enable        Enable (or disable) countdown timer
     * @param   source_clock  Source clock selection
     * @param   value         Countdown timer value
     *
     * @return  True if the given parameters were valid
     */
    bool countdown_set(bool enable, CountdownSrcClock source_clock, uint8_t value);

    /**
     * Get the current countdown timer value.
     *
     * @return  Countdown timer value (see countdown_set()
     *          comments for interpreting this)
     */
    uint8_t countdown_get();

    /**
     * Configure alarm.
     *
     * When the alarm triggers, the control register will be
     * modified and an interrupt is generated, if enabled.
     *
     * The alarm triggers when all enabled (not -1) variables
     * match the current time.
     *
     * @param   minute    Minute (0-59) on which the alarm should
     *                    trigger. Set to -1 to ignore minutes.
     *
     * @param   hour      Hour (0-23) on which the alarm should
     *                    trigger. Set to -1 to ignore hours.
     *
     * @param   day       Day (0-31) on which the alarm should
     *                    trigger. Set to -1 to ignore days.
     *
     * @param   weekday   Weekday (0-6) on which the alarm shoould
     *                    trigger. Set to -1 to ignore weekdays.
     *
     * @return  True if the given parameters were valid
     */
    bool alarm_set(int minute, int hour, int day, int weekday);

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
