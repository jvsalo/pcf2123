/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Jaakko Salo
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

  public:
    /**
     * Construct a new instance of the driver.
     *
     * @param   ce_pin  Chip enable pin for this PCF2123
     */
    PCF2123(uint8_t ce_pin);

    /**
     * Get current time of the RTC.
     */
    clock_get();

    /**
     * Set current time of the RTC.
     */
    clock_set();
};

#endif
