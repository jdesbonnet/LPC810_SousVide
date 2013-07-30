#ifndef ONEWIRE_H_
#define ONEWIRE_H_

void ow_init(int port, int pin);
void ow_low(void);
void ow_high(void);
int ow_reset(void);
int ow_read_bit(void);
void ow_write_bit(int b);
void ow_write_byte (int data);
#endif
