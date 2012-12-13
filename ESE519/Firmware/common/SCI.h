#ifndef _SCI_H
#define _SCI_H

#define SCI_FBUS F_BUS
#define BAUDH 0
#define BAUDL (SCI_FBUS / 16 / 115200)

#pragma CODE_SEG DEFAULT

extern UINT8 SCIData;
extern UINT8 SCIDataFlag;

void SCIInit(void);
void SCITx(UINT8 cData);
void SCITxStr(char *pStr);
void SCITxBuf(UINT8 *buf, UINT8 length);
void SCITxPkt(void *data, UINT8 length);

UINT8 SCIRx(void);

#pragma CODE_SEG __NEAR_SEG NON_BANKED
interrupt void SCIRx_vect();
#pragma CODE_SEG DEFAULT

#endif
