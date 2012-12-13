/* TORCS - CAN gateway
 *
 * Receives car params from TORCS and sends them on the CAN bus
 * Receives driver inputs from the CAN bus and sends them to TORCS
 */

#include <hidef.h>      /* common defines and macros */
#include <MC9S12C128.h> /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12c128"
#include <string.h>
#include <stdlib.h>
#include "common.h"
#include "CAN.h"
#include "SCI.h"
#include "types.h"
#include "PLL.h"

CarParams carParams;  //speed | engineRPM | wheel speed (FL,FR,RL,RR) | yaw rate
StabMsg stabmsg;  //lateral speed  |  lateral acceleration
volatile TorcsInput torcsInput; //accel | brake (FL,FR,RL,RR) | steer | gear | clutch
volatile UINT8 carInputsUpdated = 0;
volatile UINT8 carParamsUpdated = 0;
volatile UINT8 accelCorrection = 0;
volatile UINT8 carInputsUpdatedwii = 0; 
volatile INT8 gear = 0, Gear_temp = 0, setDist = 0;

UINT8 serialRxBuffer[17];

/*CC Parameters*/
INT8 getGear(INT8 currentGear);
#define KP_NUM (5)
#define KP_DEN (1)
#define KI_NUM (1)
#define KI_DEN (100)
CarInputs carInputs;
void init(void);

/*ABS Parameters*/
BrakeMsg brake_msg = {0,0,0,0};
volatile float y1;
volatile float y2;
volatile float y3;
volatile float y4;
volatile float y4_prev;
typedef struct _ParamTransmit {
   UINT8 accl;
   UINT8 brake;
   INT8 speed;
   UINT16 distance;
   INT8 error;
   INT16 errorInteg;
   //INT8 setSpeed;
}ParamTransmit;
ParamTransmit paramTx;

typedef struct _ParamTransmit2 {
   INT8 radarError;
   INT8 currentTrackID;
   UINT8 angle;
   INT8 isTrack;
   INT8 carYaw;
}ParamTransmit2;
ParamTransmit2 paramTx2;

typedef struct _Car
{
    float gearRatio[8];
    float enginerpmRedLine; /* rad/s */
    float SHIFT; /* = 0.95 (% of rpmredline) */
    UINT16 SHIFT_MARGIN;
    float wheelRadius[4];
} Car;
const Car car = {
    {0, 3.9*4.5, 2.9*4.5, 2.3*4.5, 1.87*4.5, 1.68*4.5, 1.54*4.5, 1.46*4.5},
    1958.26,
    0.95,
    4,
    {0.3024, 0.3024, 0.3151, 0.3151}
};
#pragma CODE_SEG __NEAR_SEG NON_BANKED

interrupt 20 void SCIRx_vect(void)   //receive messages from serial port
{
    UINT8 status, dummy;
    static UINT8 serialRxState = 0;
    static UINT8 serialData;
    static UINT8 serialDataLength = 0;
    static UINT8 serialRxChksum = 0;
    static UINT8 *rxPtr;
    
    status = SCISR1;
    
    if(SCISR1_RDRF == 0)  // No data 
        return;

    //Check for Errors (Framing, Noise, Parity) 
    if( (status & 0x07) != 0 )
    {
        dummy = SCIDRL;
        return;
    } 

    // Good Data 
    serialData = SCIDRL; // load SCI register to data 
    SCIDataFlag = 1;

    switch(serialRxState)   //serialdata (AA | CC | ...)
    {
        case 0:
            if(serialData == 0xAA)
            {
                serialRxChksum = 0xAA;  
                serialRxState = 1;
                
            }
            break;

        case 1:
            if(serialData == 0xCC && serialRxState == 1)
            {
                serialDataLength = 17;
                serialRxChksum ^= 0xCC;
                rxPtr = serialRxBuffer;
                serialRxState = 2;
            }
            else
            {
                serialRxState = 0;
            }
            break;

        case 2:
            if(serialDataLength > 0)      //copy data to serial buffer
            {
                
                
                *rxPtr = serialData;
                serialRxChksum ^= serialData;
                rxPtr++;
                serialDataLength--;
            }
            else
            {
                if(serialData == serialRxChksum)
                {
                    if(!carParamsUpdated) // Only update when old value has been used
                    {
                        memcpy(&carParams, serialRxBuffer, sizeof(CarParams));
                        //memcpy(&stabmsg, serialRxBuffer + 8, sizeof(StabMsg));
                        carParamsUpdated = 1;
                    }
                }
                serialRxState = 0;
            }
            break;
    }
}
#pragma CODE_SEG __NEAR_SEG NON_BANKED
interrupt 38 void CANRx_vect(void)     //receive messages via CAN bus
{
    UINT16 identifier;
    UINT8 length;
    identifier = (CANRXIDR0 << 3) + (CANRXIDR1 >> 5);
    
    length = CANRXDLR & 0x0F;

    if(identifier == CAN_INPUT_MSG_ID)     //messages from wiimote
    {
       
        if(length > sizeof(CarInputs))
            length = sizeof(CarInputs);

        memcpy(&carInputs, &CANRXDSR0, length);
        torcsInput.steer = carInputs.steer;  //update steer information
        setDist = carInputs.cruisedist;
        carInputsUpdated = 1;
        carInputsUpdatedwii = 1;
    }
    else; 
    /*if(identifier == CAN_BRAKE_MSG_ID) //messages from ABS ECU (brake message)
    {
        BrakeMsg brakeMsg;

        if(length > sizeof(BrakeMsg))
            length = sizeof(BrakeMsg);

        memcpy(&brakeMsg, &CANRXDSR0, length);
        torcsInput.brakeFL = brakeMsg.brakeFL;  //update brake information
        torcsInput.brakeFR = brakeMsg.brakeFR;
        torcsInput.brakeRL = brakeMsg.brakeRL;
        torcsInput.brakeRR = brakeMsg.brakeRR;
        carInputsUpdated = 1;
    } */
    /*else if(identifier == CAN_ACCEL_MSG_ID) //corrected acceleration messages
    {
        AccelMsg accelMsg;
        if(length > sizeof(AccelMsg))
            length = sizeof(AccelMsg);

        memcpy(&accelMsg, &CANRXDSR0, length);
        torcsInput.accel = accelMsg.accel;    //update acceleration, gear, clutch information
        torcsInput.gear = accelMsg.gear;
        torcsInput.clutch = accelMsg.clutch;
        carInputsUpdated = 1;
    }        */
    CANRFLG_RXF = 1; // Reset the flag
}

#pragma CODE_SEG DEFAULT
INT8 getGear(INT8 currentGear)
{
    float gr_up, omega, wr;
    float rearWheelSpeed;
    if(Gear_temp == 2)
        return 2;
    if (currentGear <= 0)
        return 1;
     

    //rearWheelSpeed = (carParams.wheelSpeedRL + carParams.wheelSpeedRR)/2;
    rearWheelSpeed = carParams.speed;

    gr_up = car.gearRatio[currentGear];
    omega = car.enginerpmRedLine/gr_up;
    wr = car.wheelRadius[2];
    if (rearWheelSpeed > omega*wr*car.SHIFT)
    {
        return currentGear + 1;
    }
    else
    {
        float gr_down = car.gearRatio[currentGear - 1];
        omega = car.enginerpmRedLine/gr_down;
        if ( (currentGear > 1) && (rearWheelSpeed + car.SHIFT_MARGIN < omega*wr*car.SHIFT) )
            return currentGear - 1;
    }
    return currentGear;
}




void main(void)
{
    INT8 setSpeed = -1;
    INT8 carSpeed = 0;
    INT16 error;
    INT16 errInteg;
    INT16 controlOutput;
    INT16 errDeriv;

    INT8 accel;

    UINT8 s_cruiseOn = 0;
    UINT8 d_cruiseOn = 0;
    AccelMsg accelMsg;
    /*ACC_Init*/
            float safeDist, mu = 1.2, g = 9.81;
            float oppSpeed = carParams.oppSpeed;
            float mySpeed = carParams.speed;
            float distance = carParams.distance;
            float disttoMiddle = carParams.disttoMiddle/10.0;
            INT8 trackFlg=0;

    //DDRB = 0xF0;
    //PORTB = 0xF0;
    //UINT16 i;

    
    init();

    EnableInterrupts;
    
    for(;;)
    {
        if(carParamsUpdated)   //send simulator's data on CAN bus
        {
            safeDist, mu = 1.2, g = 9.81;
            oppSpeed = carParams.oppSpeed;
            errDeriv = carParams.speed - mySpeed; 
            mySpeed = carParams.speed;
            distance = carParams.distance;
            disttoMiddle = carParams.disttoMiddle/10.0;
            /*ACC_Track*/
            if(carInputs.controls & TRACK){
              if(distance<=400 && carParams.isTrackable) trackFlg = 1;
              else trackFlg = 0;
            }
            
            /*No ACC*/
            if(1)
            {
                if(carInputs.gear != -1)
                    gear = (INT8) limit(getGear(gear), 0, 7);
                else
                    gear = -1;

                accelMsg.accel = (UINT8) limit(carInputs.accel - accelCorrection, 0, 100);
                accelMsg.clutch = carInputs.clutch;
                accelMsg.gear = gear;
            }
            /*ACC_Speed_Cruise*/
            if(s_cruiseOn)
            {
                error = (setSpeed - carParams.speed);
                errInteg += error;
                errInteg = limit(errInteg, -100*KI_DEN/KI_NUM, 100*KI_DEN/KI_NUM);
                controlOutput = KP_NUM*error/KP_DEN + KI_NUM*errInteg/KI_DEN- errDeriv*100;
                controlOutput = limit(controlOutput, -100, 100);

                if(controlOutput>=0){
                  carInputs.brake = 0;
                  accel = (UINT8) controlOutput;
                } else{
                  carInputs.brake = 0 - controlOutput;
                  accel = 0;
                }

                gear = (UINT8) limit(getGear(gear), 0, 7);

                accelMsg.accel = (UINT8)limit(accel - accelCorrection, 0, 100) ;
                accelMsg.gear = gear;
                accelMsg.clutch = 0;
            }
            
            /*ACC_Distance_Cruise*/
            if(d_cruiseOn && !s_cruiseOn)
            {
 
                if(trackFlg){
                  
                if(distance>900)  distance = distance - 1908;
                if(distance>setDist + 50){
                    setSpeed = carParams.oppSpeed * 1.2;
                } else  if((distance>setDist+10) && (distance<setDist+50)){
                  if(oppSpeed<10) setSpeed = 10;
                  else setSpeed = carParams.oppSpeed * 1.1;
                }
                else if((distance<setDist+10)&&(distance>setDist-10)){
                  if(oppSpeed<5) setSpeed = 5;
                  else setSpeed = oppSpeed;
                }
                else if((distance<setDist-10)&&(distance>setDist/2))  setSpeed = oppSpeed*0.8;
                else if(distance<setDist/2) setSpeed = 0.3*oppSpeed;
                
                
                }
                if(setSpeed != -1){
                  
                error = (setSpeed - carParams.speed);
                errInteg += error;
                errInteg = limit(errInteg, -100*KI_DEN/KI_NUM, 100*KI_DEN/KI_NUM);

                controlOutput = KP_NUM*error/KP_DEN + KI_NUM*errInteg/KI_DEN- errDeriv*100;
                controlOutput = limit(controlOutput, -100, 100);

                if(controlOutput>=0){
                  carInputs.brake = 0;
                  accel = (UINT8) controlOutput;
                } else{
                  carInputs.brake = 0 - controlOutput;
                  accel = 0;
                }

                gear = (UINT8) limit(getGear(gear), 0, 7);

                accelMsg.accel = (UINT8)limit(accel - accelCorrection, 0, 100) ;
                accelMsg.gear = gear;
                accelMsg.clutch = 0;
                }
            }
            
            //CANTx(CAN_ACCEL_MSG_ID, &accelMsg, sizeof(AccelMsg));
            //CANTx(CAN_PARAM_MSG_ID, &carParams, sizeof(CarParams));
            //CANTx(CAN_STAB_MSG_ID, &stabmsg, sizeof(StabMsg));
            torcsInput.accel = accelMsg.accel;    //update acceleration, gear, clutch information
            torcsInput.gear = accelMsg.gear;
            torcsInput.clutch = accelMsg.clutch;
            

            /*ABS*/
            
              
            brake_msg.brakeFL = carInputs.brake;
            brake_msg.brakeFR = carInputs.brake;
            brake_msg.brakeRL = carInputs.brake;
            brake_msg.brakeRR = carInputs.brake;
            if(carInputs.controls & ABS){
            y1= ((INT8)((carParams.engineRPM & 0xFF00)>>8))/50;      //lat vel
            y2= ((INT8)(carParams.yawRate))*(180/3.142)/20; // yaw rate
            y4= ((INT8)(carParams.engineRPM & 0xFF))/100; //roll angle (k)
            y3= y4-y4_prev;              // rollr rate (difference)
            y4_prev=y4;
            brake_msg.brakeFL= limit(1.5335*(-y1/0.10859) -2.3662*(y2/1.231) -7.2193*(-y3/0.041089) +3.7494*(y4/0.5834),0,100);
            brake_msg.brakeRL=brake_msg.brakeFL;
            
            brake_msg.brakeFR= limit(-1.5081*(-y1/0.10859) +2.3269*(y2/1.231) +7.0995*(-y3/0.041089) -3.6872*(y4/0.5834),0,100);
            brake_msg.brakeRR=brake_msg.brakeFR;
            }
            torcsInput.brakeFL = brake_msg.brakeFL;  //update brake information
            torcsInput.brakeFR = brake_msg.brakeFR;
            torcsInput.brakeRL = brake_msg.brakeRL;
            torcsInput.brakeRR = brake_msg.brakeRR;
            carInputsUpdated = 1;
            paramTx.accl = torcsInput.accel;
            paramTx.brake = carInputs.brake;
            paramTx.speed = carParams.speed;
            paramTx.distance = carParams.distance;
            paramTx.error = error;
            paramTx.errorInteg = errInteg;
            paramTx2.currentTrackID = carParams.currentTrackID;
            paramTx2.radarError = carParams.radarError;
            paramTx2.angle = carParams.angle;
            paramTx2.isTrack = trackFlg;
            paramTx2.carYaw = carParams.carYaw;
            //paramTx.setSpeed = setSpeed;
            CANTx(CAN_PARAM_MSG_ID, &paramTx, sizeof(ParamTransmit));
            CANTx(CAN_PARAM_MSG_ID2, &paramTx2, sizeof(ParamTransmit2));
            carParamsUpdated = 0;
        }

        if(carInputsUpdated)   //send wiimote's data to simulator via serial port
        {
            TorcsInput temp;
              
            if(carInputs.controls & S_CRUISE)
            {
                if(!trackFlg) errInteg = 0;
                setSpeed = carParams.oppSpeed;
                if(!s_cruiseOn)
                {
                    
                    s_cruiseOn = 1;
                    errInteg = 0;
                    PORTB = 0x70;
                }
            }
            else
            {
                s_cruiseOn = 0;
                setSpeed = -1;
                PORTB = 0xF0;
            }
            if(carInputs.controls & D_CRUISE){
              if(!trackFlg) errInteg = 0;
              if(!d_cruiseOn)
                {
                    setDist = carInputs.cruisedist;
                    d_cruiseOn = 1;
                    errInteg = 0;
                    PORTB = 0x70;
                }
            } else{
              d_cruiseOn = 0;
              setDist = 0;
              setSpeed = -1;
            }
            if(!s_cruiseOn&&!d_cruiseOn)
            {
                if(carInputs.gear != -1)
                    gear = (UINT8)limit(getGear(gear), 0, 7);
                else
                    gear = -1;

                accelMsg.accel = (UINT8)limit(carInputs.accel - accelCorrection, 0, 100);
                accelMsg.clutch = carInputs.clutch;
                accelMsg.gear = gear;
            }
            memcpy(&temp, &torcsInput, sizeof(TorcsInput));
            SCITxPkt(&temp, sizeof(TorcsInput));
            carInputsUpdated = 0;
        }
    }
}

void init()
{
    setclk24();    // set Eclock to 24 MHZ
    SCIInit();
    CANInit();
}
