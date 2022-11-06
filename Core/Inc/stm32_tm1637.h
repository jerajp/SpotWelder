#ifndef __stm32_tm1637_H__
#define __stm32_tm1637_H__

void tm1637Init(void);
void tm1637DisplayDecimal(int v, int displaySeparator);
void tm1637SetBrightness(char brightness);

#endif
