#ifndef _CAN_H
#define _CAN_H

void CANInit(void);

INT8 CANTx(UINT16 identifier, void *data, UINT8 length);

INT8 CANRx(UINT16 *identifier, void *buffer);

#endif
