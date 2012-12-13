#ifndef _TYPES_H
#define _TYPES_H

#define CAN_PARAM_MSG_ID (0x70) /* Car params from simulator */
#define CAN_PARAM_MSG_ID2 (0x71) /* Car params from simulator */
#define CAN_INPUT_MSG_ID (0x60) /* Raw driver input */
#define CAN_BRAKE_MSG_ID (0x61) /* Computed brake output */
#define CAN_ACCEL_MSG_ID (0x62) /* Computed accel, gear and clutch values */
#define CAN_ACCEL_CORR_MSG_ID (0x80) /* Accelerator correction from EBCM to ECM for TC */
#define CAN_STAB_MSG_ID (0x84) /* Used for stability control */


#define D_CRUISE (0x08)
#define ABS       (0x04)
#define TRACK  (0x02)
#define S_CRUISE    (0x01)

typedef struct _CarParams
{
    INT8 speed;
    UINT16 engineRPM;
    INT8 wheelSpeedFL;
    INT8 wheelSpeedFR;
    INT8 wheelSpeedRL;
    INT8 wheelSpeedRR;
    INT8 yawRate;
    INT8 disttoMiddle;
    INT8 oppSpeed;
    UINT16 distance;
    INT8 isTrackable;
    INT8 radarError;
    INT8 currentTrackID;
    UINT8 angle;
    INT8 carYaw;
    //INT8 lateralspeed;
    //INT8 lateralacc;
} CarParams;

typedef struct _StabMsg 
{
    INT8 lateralspeed;
    INT8 lateralacc;
} StabMsg;

#define SERIAL_CAR_PARAMS_PKT_LENGTH (2 + sizeof(CarParams) + sizeof(StabMsg) + 1)

typedef struct _CarInputs {
    UINT8 accel;
    UINT8 brake;
    INT8 steer;
    INT8 gear;
    UINT8 clutch;
    UINT8 controls; /* use it for ABS/TRAC/CRUISE  */
    UINT8 cruisedist;
} CarInputs;
#define SERIAL_CAR_INPUTS_PKT_LENGTH (2 + sizeof(CarInputs) + 1)

typedef struct _AccelMsg
{
    UINT8 accel;
    INT8 gear;
    UINT8 clutch;
} AccelMsg;

typedef struct _BrakeMsg
{
    UINT8 brakeFL;
    UINT8 brakeFR;
    UINT8 brakeRL;
    UINT8 brakeRR;
} BrakeMsg;

typedef struct _TorcsInput
{
    UINT8 accel;
    UINT8 brakeFL;
    UINT8 brakeFR;
    UINT8 brakeRL;
    UINT8 brakeRR;
    INT8 steer;
    INT8 gear;
    UINT8 clutch;
} TorcsInput;


  

#endif
