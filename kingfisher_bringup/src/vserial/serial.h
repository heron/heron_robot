#ifndef SERIAL_H_
#define SERIAL_H_

int OpenSerial(int* fd, const char* port_name);

int SetupSerial(int fd, int baudrate);

#endif /* SERIAL_H_ */
