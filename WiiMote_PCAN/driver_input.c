/* PCAN - driver input
 *
 * Transmits driver inputs from the wiimote and sends them on the CAN bus via PCAN
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <wiiuse.h>
#include <libpcan.h>
#include <unistd.h>
#include <stdint.h>
#include <ctype.h>
#include <fcntl.h>
#include <errno.h>

#define PCAN_DEVICE "/dev/pcanusb0"
#define CAN_BAUD_RATE_REG_VAL 0x001C  // 500kbps

#define MAX_WIIMOTES    1
#define CAN_INPUT_MSG_ID (0x60) /* Raw driver input */

#define STABILITY   0x08
#define ABS         0x04
#define TC          0x02
#define CRUISE      0x01

typedef struct {
    uint8_t accel;
    uint8_t brake;
    int8_t steer;
    int8_t gear;
    uint8_t clutch;
    uint8_t controls; /* used for ABS/TRAC/CRUISE  */
    uint8_t cruisedist;
} CarInputs;

CarInputs carInputs = {0, 0, 0, 0, 0, 0, 50};

HANDLE initCAN(void);

void handle_event(struct wiimote_t* wm)
{
    static uint8_t controls = ABS | TC | STABILITY;
#if 0
    printf("\n\n--- EVENT [id %i] ---\n", wm->unid);

    /* if a button is pressed, report it */
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_A))		printf("A pressed\n");
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_B))		printf("B pressed\n");
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_UP))		printf("UP pressed\n");
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_DOWN))	printf("DOWN pressed\n");
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_LEFT))	printf("LEFT pressed\n");
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_RIGHT))	printf("RIGHT pressed\n");
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_MINUS))	printf("MINUS pressed\n");
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_PLUS))	printf("PLUS pressed\n");
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_ONE))		printf("ONE pressed\n");
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_TWO))		printf("TWO pressed\n");
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_HOME))	printf("HOME pressed\n");
#endif
    /*
     *	Pressing home will tell the wiimote we are interested in movement.
     */
    if (IS_JUST_PRESSED(wm, WIIMOTE_BUTTON_HOME))
    {
        wiiuse_set_orient_threshold(wm, 0.5);
        wiiuse_motion_sensing(wm, 1);
    }

    if(IS_JUST_PRESSED(wm, WIIMOTE_BUTTON_PLUS)){
	carInputs.gear++;
	if(carInputs.gear>7)	carInputs.gear = 7;
        controls ^= ABS;
	}
    if(IS_JUST_PRESSED(wm, WIIMOTE_BUTTON_UP)){
	carInputs.cruisedist = carInputs.cruisedist +10;
	if(carInputs.cruisedist > 100) carInputs.cruisedist = 100;
    }
    if(IS_JUST_PRESSED(wm, WIIMOTE_BUTTON_DOWN)){
	carInputs.cruisedist = carInputs.cruisedist -10;
        if(carInputs.cruisedist <50) carInputs.cruisedist = 50;
    }
    if(IS_JUST_PRESSED(wm, WIIMOTE_BUTTON_MINUS)){
	if(carInputs.gear>=2)
	carInputs.gear--;

        controls ^= TC;
	}
    if (IS_JUST_PRESSED(wm, WIIMOTE_BUTTON_RIGHT)){
	controls ^= STABILITY;
	carInputs.gear = 0;
    	}

    if(IS_JUST_PRESSED(wm, WIIMOTE_BUTTON_A))
        controls ^= CRUISE;

    if (WIIUSE_USING_ACC(wm))
    {
        // obtain the steering angle input from wiimote
        float steer_temp =  wm->orient.pitch;
        if(steer_temp < -90)
            steer_temp = -90;
        else if(steer_temp > 90)
            steer_temp = 90;

        if(steer_temp != 0)
            carInputs.steer = (steer_temp/abs(steer_temp))*pow(steer_temp/90,2)*50;
        else
            carInputs.steer = 0;
        //printf("steer: %d\n", carInputs.steer);
    }

    // obtain acceleration input from WiiMote
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_ONE))
    {
        carInputs.accel = 100;
    }
    else
        carInputs.accel = 0;

    // obtain brake input from WiiMote
    if (IS_PRESSED(wm, WIIMOTE_BUTTON_TWO))
    {
        carInputs.brake = 100;
        controls &= ~(CRUISE);
    }
    else
        carInputs.brake = 0;

    if (IS_PRESSED(wm, WIIMOTE_BUTTON_LEFT))
        carInputs.gear = -1;
    carInputs.clutch = 0;
    carInputs.controls = controls;
}

/**
 *	@brief Callback that handles a disconnection event.
 *
 *	@param wm				Pointer to a wiimote_t structure.
 *
 *	This can happen if the POWER button is pressed, or
 *	if the connection is interrupted.
 */
void handle_disconnect(wiimote* wm) {
    printf("\n\n--- DISCONNECTED [wiimote id %i] ---\n", wm->unid);
}

int main(void)
{
    wiimote *wm;
    wiimote** wiimotes;
    int found, connected;
    HANDLE canHandle;
    TPCANMsg Message;
    int wiimote_led_state;
    int exit = 0;

    canHandle = initCAN();
    if(canHandle == NULL)
    {
        printf("Error opening CAN device!\n");
        return -1;
    }

    wiimotes =  wiiuse_init(MAX_WIIMOTES);

    found = wiiuse_find(wiimotes, MAX_WIIMOTES, 5);
    if (!found)
    {
        printf ("No wiimotes found.\n");
        return 0;
    }

    connected = wiiuse_connect(wiimotes, MAX_WIIMOTES);
    if (connected)
        printf("Connected to %i wiimotes (of %i found).\n", connected, found);
    else
    {
        printf("Failed to connect to any wiimote.\n");
        return 0;
    }

    wm = wiimotes[0];
    wiiuse_status(wm);
    while(wm->event != WIIUSE_STATUS)
    {
        wiiuse_poll(wiimotes, MAX_WIIMOTES);
    }
    printf("Battery level: %f%%\n", wm->battery_level*100);

    while (1)
    {
        if(exit)
            break;
        if (wiiuse_poll(wiimotes, MAX_WIIMOTES))
        {
            /*
             *	This happens if something happened on any wiimote.
             *	So go through each one and check if anything happened.
             */
            int i = 0;
            for (; i < MAX_WIIMOTES; ++i)
            {
                switch (wiimotes[i]->event)
                {
                    case WIIUSE_EVENT:
                        /* a generic event occured */
                        handle_event(wiimotes[i]);
                        Message.ID = CAN_INPUT_MSG_ID;
                        Message.MSGTYPE = MSGTYPE_STANDARD;
                        Message.LEN = 7;
                        Message.DATA[0] = carInputs.accel;
                        Message.DATA[1] = carInputs.brake;
                        Message.DATA[2] = carInputs.steer;
                        Message.DATA[3] = carInputs.gear;
                        Message.DATA[4] = carInputs.clutch;
                        Message.DATA[5] = carInputs.controls;
			Message.DATA[6] = carInputs.cruisedist;
                        CAN_Write(canHandle,&Message);

                        // Show the status of ABS/TC/Cruise on the LEDs
                        wiimote_led_state = 0;
                        if(carInputs.controls & ABS)
                            wiimote_led_state |= WIIMOTE_LED_1;
                        if(carInputs.controls & TC)
                            wiimote_led_state |= WIIMOTE_LED_2;
                        if(carInputs.controls & STABILITY)
                            wiimote_led_state |= WIIMOTE_LED_3;
                        if(carInputs.controls & CRUISE)
                            wiimote_led_state |= WIIMOTE_LED_4;
                        wiiuse_set_leds(wm, wiimote_led_state);

                        break;
                    case WIIUSE_DISCONNECT:
                    case WIIUSE_UNEXPECTED_DISCONNECT:
                        /* the wiimote disconnected */
                        handle_disconnect(wiimotes[i]);
                        exit = 1;
                        break;
                    default:
                        break;
                }
            }
        }
    }
    wiiuse_cleanup(wiimotes, MAX_WIIMOTES);
    return 0;

}

HANDLE initCAN(void)
{
    const char *szDevNode = PCAN_DEVICE;
    uint16_t wBTR0BTR1 = CAN_BAUD_RATE_REG_VAL;
    HANDLE h;

    h = LINUX_CAN_Open(szDevNode, O_RDWR);
    if (!h)
    {
        printf("transmitest: can't open %s\n", szDevNode);
        return NULL;
    }
    // init to a user defined bit rate
    if (wBTR0BTR1)
    {
        errno = CAN_Init(h, wBTR0BTR1, CAN_INIT_TYPE_ST);
        if (errno)
        {
            perror("transmitest: CAN_Init()");
            return NULL;
        }
    }
    return h;
}
