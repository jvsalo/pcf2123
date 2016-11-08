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

#define CMD_READ  1
#define CMD_WRITE 0
#define CMD_BYTE(RW, ADDR) (0x10 | ((RW) ? 0x80 : 0x00) | (ADDR))

#define REG_CTRL1_ADDR            0x00
#define REG_CTRL2_ADDR            0x01
#define REG_TIME_DATE_ADDR        0x02
#define REG_ALARM_ADDR            0x09
#define REG_OFFSET_ADDR           0x0D
#define REG_TIMER_CLKOUT_ADDR     0x0E
#define REG_COUNTDOWN_TIMER_ADDR  0x0F

bool
PCF2123_CtrlRegs::get(int bit)
{
  return this->ctrl[bit/8] & (1 << (bit%8));
}

bool
PCF2123_CtrlRegs::set(int bit, bool value)
{
  bool old = this->get(bit);

  if (value) this->ctrl[bit/8] |=  (1 << (bit%8));
  else       this->ctrl[bit/8] &= ~(1 << (bit%8));

  return old;
}

void
PCF2123_CtrlRegs::mask_alarms()
{
  this->set(MSF, true);
  this->set(AF, true);
  this->set(TF, true);
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

  for (size_t i = 0; i < sz; i++)
    buf[i] = SPI.transfer(buf[i]);

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

PCF2123::PCF2123(uint8_t ce_pin)
  : ce_pin_(ce_pin), spi_cfg_(SPI_MAX_SPEED, MSBFIRST, SPI_MODE0)
{
  SPI.begin();

  pinMode(ce_pin_, OUTPUT);
  digitalWrite(ce_pin_, LOW);

  /* Make sure the clock is in 24h mode */
  PCF2123_CtrlRegs regs = this->ctrl_get();
  regs.set(PCF2123_CtrlRegs::HOUR_MODE, 0);
  this->ctrl_set(&regs, true, false, true);
}

bool
PCF2123::time_get(tmElements_t *now)
{
  uint8_t buf[1+7];

  buf[0] = CMD_BYTE(CMD_READ, REG_TIME_DATE_ADDR);
  this->rxt(buf, sizeof(buf));

  now->Second = bcd_decode(buf[1] & ~0x80);
  now->Minute = bcd_decode(buf[2] & ~0x80);
  now->Hour   = bcd_decode(buf[3] & ~0xC0); /* 24h clock */
  now->Day    = bcd_decode(buf[4] & ~0xC0);
  now->Wday   = bcd_decode(buf[5] & ~0xF8);
  now->Month  = bcd_decode(buf[6] & ~0xE0);
  now->Year   = bcd_decode(buf[7]);

  return !(buf[1] & 0x80);
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
  buf[5] = bcd_encode(new_time->Wday);
  buf[6] = bcd_encode(new_time->Month);
  buf[7] = bcd_encode(new_time->Year);

  this->rxt(buf, sizeof(buf));
}

void
PCF2123::reset()
{
  uint8_t buf[1+1];
  buf[0] = CMD_BYTE(CMD_WRITE, REG_CTRL1_ADDR);
  buf[1] = 0x58;
  this->rxt(buf, sizeof(buf));
}

void
PCF2123::stop(bool stopped)
{
  PCF2123_CtrlRegs regs = ctrl_get();
  regs.set(PCF2123_CtrlRegs::STOP, stopped);
  this->ctrl_set(&regs, true,  /* Write ctrl1 */
                        false, /* No ctrl2 */
                        true   /* Mask alarms */);
}

bool
PCF2123::clkout_freq_set(uint16_t freq)
{
  uint8_t COF;
  uint8_t buf[1+1];

  switch (freq)
  {
    case 0:     COF = 7; break;
    case 1:     COF = 6; break;
    case 1024:  COF = 5; break;
    case 2048:  COF = 4; break;
    case 4096:  COF = 3; break;
    case 8192:  COF = 2; break;
    case 16384: COF = 1; break;
    case 32768: COF = 0; break;
    default: return false;
  }

  buf[0] = CMD_BYTE(CMD_READ, REG_TIMER_CLKOUT_ADDR);
  this->rxt(buf, sizeof(buf));

  buf[0] = CMD_BYTE(CMD_WRITE, REG_TIMER_CLKOUT_ADDR);
  buf[1] &= ~0x70;
  buf[1] |= COF << 4;
  this->rxt(buf, sizeof(buf));

  return true;
}

bool
PCF2123::countdown_set(bool enable, CountdownSrcClock source_clock, uint8_t value)
{
  uint8_t buf[1+2];

  if (source_clock < 0 || source_clock > 3)
    return false;

  buf[0] = CMD_BYTE(CMD_READ, REG_TIMER_CLKOUT_ADDR);
  this->rxt(buf, 2);

  /* First disable the countdown timer. */
  buf[0] = CMD_BYTE(CMD_WRITE, REG_TIMER_CLKOUT_ADDR);
  buf[1] &= ~0x08;
  this->rxt(buf, 2);

  /* The previous clobbered buf[1], so re-read */
  buf[0] = CMD_BYTE(CMD_READ, REG_TIMER_CLKOUT_ADDR);
  this->rxt(buf, 2);

  /* Reconfigure timer */
  buf[0] = CMD_BYTE(CMD_WRITE, REG_TIMER_CLKOUT_ADDR);
  buf[1] = (buf[1] & ~0x08) | ((uint8_t)enable << 3);
  buf[1] = (buf[1] & ~0x03) | source_clock;
  buf[2] = value;
  this->rxt(buf, 3);

  return true;
}

uint8_t
PCF2123::countdown_get()
{
  uint8_t buf[1+1];
  buf[0] = CMD_BYTE(CMD_READ, REG_COUNTDOWN_TIMER_ADDR);
  this->rxt(buf, sizeof(buf));
  return buf[1];
}

bool
PCF2123::alarm_set(int minute, int hour, int day, int weekday)
{
  uint8_t buf[1+4];

  if ((minute < 0 || minute > 59) && minute != -1) return false;
  if ((hour < 0 || hour > 23) && hour != -1) return false;
  if ((day < 0 || day > 31) && day != -1) return false;
  if ((weekday < 0 || weekday > 6) && weekday != -1) return false;

  buf[0] = CMD_BYTE(CMD_WRITE, REG_ALARM_ADDR);
  buf[1] = minute < 0 ? 0x80 : bcd_encode(minute);
  buf[2] = hour < 0 ? 0x80 : bcd_encode(hour);
  buf[3] = day < 0 ? 0x80 : bcd_encode(day);
  buf[4] = weekday < 0 ? 0x80 : bcd_encode(weekday);
  this->rxt(buf, sizeof(buf));

  return true;
}

PCF2123_CtrlRegs
PCF2123::ctrl_get()
{
  uint8_t buf[1+2];
  PCF2123_CtrlRegs regs;

  buf[0] = CMD_BYTE(CMD_READ, REG_CTRL1_ADDR);
  this->rxt(buf, sizeof(buf));

  regs.ctrl[0] = buf[1];
  regs.ctrl[1] = buf[2];
  return regs;
}

void
PCF2123::ctrl_set(PCF2123_CtrlRegs *regs,
                  bool set_ctrl1,
                  bool set_ctrl2,
                  bool mask_alarms)
{
  uint8_t buf[1+2];
  int wrsz;

  if (mask_alarms)
    regs->mask_alarms();

  if (set_ctrl1 && set_ctrl2)
  {
    buf[0] = CMD_BYTE(CMD_WRITE, REG_CTRL1_ADDR);
    buf[1] = regs->ctrl[0];
    buf[2] = regs->ctrl[1];
    wrsz = 3;
  }

  else if (set_ctrl1)
  {
    buf[0] = CMD_BYTE(CMD_WRITE, REG_CTRL1_ADDR);
    buf[1] = regs->ctrl[0];
    wrsz = 2;
  }

  else if (set_ctrl2)
  {
    buf[0] = CMD_BYTE(CMD_WRITE, REG_CTRL2_ADDR);
    buf[1] = regs->ctrl[1];
    wrsz = 2;
  }

  else return;

  this->rxt(buf, wrsz);
}
