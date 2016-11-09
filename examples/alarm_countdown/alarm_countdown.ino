/*
  Demonstrates the use of the alarm and countdown features.

  The type tmElements_t and functions like breakTime() are from
  the Time library.
*/

#include <PCF2123.h>

PCF2123 rtc(9); /* Chip enable on pin 9 */

void setup() {
  tmElements_t tm;
  Serial.begin(57600);

  Serial.println("Resetting RTC");
  rtc.reset();

  Serial.println("Setting time to Tue, 08 Nov 2016 23:49:50");
  breakTime(1478648990, tm);
  rtc.time_set(&tm);

  Serial.println("Setting alarm at 23:50:00");
  breakTime(1478648990 + 10, tm);
  rtc.alarm_set(tm.Minute, tm.Hour, tm.Day, tm.Wday);

  Serial.println("Enabling repeating countdown timer with 5 second interval");
  rtc.countdown_set(true, PCF2123::CNTDOWN_CLOCK_1HZ, 5);
}

/* Print human readable time */
void print_time(tmElements_t t) {
  Serial.print(String("Year ") + (1970 + t.Year));
  Serial.print(String(" Month ") + t.Month);
  Serial.print(String(" Day ") + t.Day);
  Serial.print(String(" Wday ") + t.Wday);
  Serial.print(String(" Hour ") + t.Hour);
  Serial.print(String(" Minute ") + t.Minute);
  Serial.print(String(" Second ") + t.Second);
  Serial.print("\n");
}

void loop() {
  tmElements_t t;

  /* Get and print current time */
  rtc.time_get(&t);
  print_time(t);

  /* Read control registers */
  PCF2123_CtrlRegs regs = rtc.ctrl_get();

  /* Check for alarm */
  if (regs.get(PCF2123_CtrlRegs::AF))
  {
    Serial.println("Alarm triggered!");

    /* Set the alarm flag back to low */
    regs.set(PCF2123_CtrlRegs::AF, false);

    rtc.ctrl_set(
      &regs,
      false,   /* No need to update Ctrl1, AF is in Ctrl2 */
      true,    /* Update Ctrl2 */
      false);  /* Don't mask alarm flags, we are changing one! */
  }

  /* Check for countdown timer */
  if (regs.get(PCF2123_CtrlRegs::TF))
  {
    Serial.println("Countdown timer triggered!");

    /* Set the countdown timer flag back to low */
    regs.set(PCF2123_CtrlRegs::TF, false);
    rtc.ctrl_set(&regs, false, true, false);
  }

  delay(1000);
}
