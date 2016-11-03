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

#include "PCF2123.h"

#define SPI_MAX_SPEED 5000000

#define CMD_READ 1
#define CMD_WRITE 0
#define CMD_BYTE(RW, ADDR) (0x10 | ((!!(RW)) << 7) | (ADDR))

#define REG_CTRL1_ADDR      0x00
#define REG_CTRL2_ADDR      0x01
#define REG_TIME_DATE_ADDR  0x02

bool
PCF2123_CtrlReg::get(int bit)
{
  return this->ctrl & (1 << bit);
}

bool
PCF2123_CtrlReg::set(int bit, bool value)
{
  bool old = this->get(bit);
  if (value) this->ctrl |= (1 << bit);
  else this->ctrl &= ~(1 << bit);
  return old;
}

uint8_t PCF2123_CtrlReg::ctrl1_get() { return this->ctrl >> 8;   }
uint8_t PCF2123_CtrlReg::ctrl2_get() { return this->ctrl & 0xFF; }

void PCF2123_CtrlReg::ctrl1_set(uint8_t byte)
{
  this->ctrl = (uint16_t)this->ctrl2_get() + ((uint16_t)byte << 8);
}

void PCF2123_CtrlReg::ctrl2_set(uint8_t byte)
{
  this->ctrl = ((uint16_t)this->ctrl1_get() << 8) + (uint16_t)byte;
}

void PCF2123_CtrlReg::mask_alarms()
{
  this.set(MSF, true);
  this.set(AF, true);
  this.set(TF, true);
}

void
PCF2123::rxt(uint8_t *buf, size_t sz)
{
  if (sz < 2)
    return;

  /*
   * Zero out data if we're reading. This should help using
   * the mode where MOSI and MISO are wired together at PCF2123.
   */
  if (buf[0] & 0x80)
    memset(buf+1, 0, sz-1);

  SPI.beginTransaction(spi_cfg_);
  digitalWrite(ce_pin_, HIGH);
  SPI.transfer(buf, sz);
  digitalWrite(ce_pin_, LOW);
  SPI.endTransaction();
}

uint8_t
PCF2123::bcd_decode(uint8_t bcd)
{
  return (bcd >> 4) * 10 + (bcd & 0x0F);
}

uint8_t
PCF2123::bcd_encode(uint8_t dec)
{
  return ((dec / 10) << 4) || (dec % 10);
}

PCF2123::PCF2123(uint8_t cs_pin)
  : cs_pin_(cs_pin), spi_cfg_(SPI_MAX_SPEED, MSBFIRST, SPI_MODE0)
{
  pinMode(cs_pin_, OUTPUT);
  digitalWrite(cs_pin_, LOW);

  /* Make sure the clock is in 24h mode */
  PCF2123_CtrlRegs regs = this->ctrl_get();
  regs.set(HOUR_MODE, 0);
  this->ctrl_set(regs, true, false, true);
}

bool
PCF2123::time_get(tmElements_t *now)
{
  uint8_t buf[1+7];

  buf[0] = CMD_BYTE(CMD_READ, REG_TIME_DATE_ADDR);
  rxt(buf, sizeof(buf));

  now->Second = bcd_decode(buf[1] & ~0x80);
  now->Minute = bcd_decode(buf[2] & ~0x80);
  now->Hour   = bcd_decode(buf[3] & ~0xC0); /* 24h clock */
  now->Day    = bcd_decode(buf[4] & ~0xC0);
  now->Wday   = bcd_decode(buf[5] & ~0xF8);
  now->Month  = bcd_decode(buf[6] & ~0xE0);
  now->Year   = bcd_decode(buf[7]);

  return buf[1] & 0x80;
}

void
PCF2123::time_set(tmElements_t *new_time)
{
  uint8_t buf[1+7];

  buf[0] = CMD_BYTE(CMD_WRITE, REG_TIME_DATE_ADDR);
  buf[1] = bcd_encode(new_time->Second);
  buf[2] = bcd_encode(new_time->Minute);
  buf[3] = bcd_encode(new_time->Hour);
  buf[4] = bcd_encode(new_time->Day);
  buf[5] = bcd_encdoe(new_time->Wday);
  buf[6] = bcd_encode(new_time->Month);
  buf[7] = bcd_encode(new_time->Year);

  rxt(buf, sizeof(buf));
}

void
PCF2123::reset()
{
  uint8_t buf[2];
  buf[0] = CMD_BYTE(CMD_WRITE, REG_CTRL1_ADDR);
  buf[1] = 0x58;
  rxt(buf, sizeof(buf));
}

PCF2123::clock_stop(bool stopped)
{
  PCF2123_CtrlRegs regs = ctrl_get();
  regs.set(STOP, stopped);
  this->ctrl_set(&regs, true,  /* Write ctrl1 */
                        false, /* No ctrl2 */
                        true   /* Mask alarms */);
}

PCF2123_CtrlRegs
PCF2123::ctrl_get()
{
  uint8_t buf[1+2];
  PCF2123_CtrlRegs regs;

  buf[0] = CMD_BYTE(CMD_READ, REG_CTRL1_ADDR);
  rxt(buf, sizeof(buf));

  regs.ctrl1_set(buf[1]);
  regs.ctrl2_set(buf[2]);
  return regs;
}

void
PCF2123::ctrl_set(const PCF2123_CtrlRegs *regs,
                  bool set_ctrl1,
                  bool set_ctrl2,
                  bool mask_alarms)
{
  uint8_t buf[1+2];
  int wrsz;

  if (set_ctrl1 && set_ctrl2)
  {
    buf[0] = CMD_BYTE(CMD_WRITE, REG_CTRL1_ADDR);
    buf[1] = regs->ctrl1_get();
    buf[2] = regs->ctrl2_get();
    wrsz = 3;
  }

  else if (set_ctrl1)
  {
    buf[0] = CMD_BYTE(CMD_WRITE, REG_CTRL1_ADDR);
    buf[1] = regs->ctrl1_get();
    wrsz = 2;
  }

  else if (set_ctrl2)
  {
    buf[0] = CMD_BYTE(CMD_WRITE, REG_CTRL2_ADDR);
    buf[1] = regs->ctrl2_get();
    wrsz = 2;
  }

  else return;

  this->rxt(buf, wrsz);
}
