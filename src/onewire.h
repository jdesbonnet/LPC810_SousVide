#ifndef ONEWIRE_H_
#define ONEWIRE_H_

void ow_init(int port, int pin);
void ow_low(void);
void ow_high(void);
int ow_reset(void);
int ow_read(void);
int ow_bit_read(void);
void ow_bit_write(int b);
void ow_byte_write (int data);
int ow_byte_read(void);
#endif
