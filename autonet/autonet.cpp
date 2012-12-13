/***************************************************************************

file                 : autonet.cpp
created              : Sat Nov 13 23:19:31 EST 2010
copyright            : (C) 2002 Utsav

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif
#include "berniw.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>
#include "serial.h"
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <errno.h>
#include "timer.h"
#include <iostream>
using namespace std;


static tTrack *curTrack;
 int serialfd;


static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newrace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt);
int IsTrackable(int myTrackIndex, int oppTrackIndex);

static MyCar* mycar[BOTS] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };
static OtherCar* ocar = NULL;
static TrackDesc* myTrackDesc = NULL;
static double currenttime;
static int base = 201;
static const tdble waitToTurn = 1.0;
//berniw
static tCarElt *car;
static float accel = 0;
static float brake[4] = {0};
static float clutch = 0;
static float angle = 0 ;
static int gear = 0 ;
static int currentTrackID = 1;
static timer_t timerid;
volatile static bool signalStopSendData = false;
volatile static bool serialDataStopped = true;

/*
 * Module entry point
 */
extern "C" int autonet(tModInfo *modInfo)
{
    int i;


    memset(modInfo, 0, 10*sizeof(tModInfo));


    modInfo->name    = "autonet";		/* name of the module (short) */
    modInfo->desc    = "";	            /* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0;
}

/* Module interface initialization. */
static int InitFuncPt(int index, void *pt)
{
     //cout<<" Init FuncPt for index "<< index <<endl;
    tRobotItf *itf  = (tRobotItf *)pt;


    itf->rbNewTrack = initTrack; /* Give the robot the track view called */
                                 /* for every track change or new race */
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
  
    return 0;
}
/*Determine if there is turn in 2 cars*/
int IsTrackable(int myTrackIndex, int oppTrackIndex)
{
	int i;
	for(i = myTrackIndex;i!=oppTrackIndex;i++)
	{
		if(i>1908)	i-=1908;
		if(myTrackDesc->getSegmentPtr(i)->getType() == TR_LFT || myTrackDesc->getSegmentPtr(i)->getType() == TR_RGT)	return 0 ;
	}
	return 1;
}
int RandomG(){
	base = (base*17)%10000;
	return (base);
}
/*Get the ID of the nearest trackable car*/
int GetTrackableCarID(int num){
	int i = 0, id = 0, flg = 0;
	float angle1, angleMin = 90;
	int dist, distMin = 1908;
	float disttomiddle;
	for(i = 1;i<num;i++)
	{
		dist = ocar[i].getCurrentSegId() - ocar[0].getCurrentSegId();

		if(dist<0)	dist = dist + 1908;
		if(dist<distMin){
			distMin = dist;
			id = i;
		}
/*		if(IsTrackable(ocar[0].getCurrentSegId(), ocar[i].getCurrentSegId())){
			disttomiddle = ocar[i].getCarPtr()->_trkPos.toMiddle - car->_trkPos.toMiddle;			
			angle1 = atan(disttomiddle/dist);
			angle1 = angle1 + RtTrackSideTgAngleL(&(car->_trkPos))-car->_yaw;
			angle1 = angle1/3.141593*180 - 360;
			if(angle1<0)	angle1 = 0 - angle1;
			else;
			if(angle1<angleMin){
				angleMin = angle1;
				id = i;
			}
			flg = 1;
		}
		else{
			if(dist<distMin){
				if(flg == 0){
					distMin = dist;
					id = i;
				}
			}
		}*/
			
	}
	return id;
}
/* Called for every track change or new race. */
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
	if ((myTrackDesc != NULL) && (myTrackDesc->getTorcsTrack() != track)) {
		delete myTrackDesc;
		myTrackDesc = NULL;
	}
	if (myTrackDesc == NULL) {
		myTrackDesc = new TrackDesc(track);
	}

	char buffer[BUFSIZE];
	char* trackname = strrchr(track->filename, '/') + 1;

	sprintf(buffer, "drivers/berniw/%d/%s", index, trackname);
    *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);

	if (*carParmHandle == NULL) {
		sprintf(buffer, "drivers/berniw/%d/default.xml", index);
	    *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
	}
	float fuel = GfParmGetNum(*carParmHandle, BERNIW_SECT_PRIV, BERNIW_ATT_FUELPERLAP,
		(char*)NULL, track->length*MyCar::MAX_FUEL_PER_METER);
	fuel *= (s->_totLaps + 1.0);
	GfParmSetNum(*carParmHandle, SECT_CAR, PRM_FUEL, (char*)NULL, MIN(fuel, 100.0));
    //*carParmHandle = NULL;
 
}

void serialData(int signum)
{
    uint8_t a[512]; // Large buffer to read in all pending bytes from serial
    uint8_t calcChkSum = 0;
    uint8_t b[20];
    int i;
    //cout<<"Signal Number is " << signum << endl;

    #if 1
    if(signalStopSendData)
    {
         
        serialDataStopped = true;
        return;
    }

    if(serialPortRead(serialfd, a, 1) == 1) {
       // cout<<" BLAH " << endl; 
        if(a[0] == 0xAA) {
          // cout<<"Got AA " << endl;
            if(serialPortRead(serialfd, a, 1) == 1) {
                if(a[0] == 0xCC) {
                   // cout<<"Got CC " << endl;
                    calcChkSum = 0xAA ^ 0xCC;
                    if(serialPortRead(serialfd, a, 512) >= 9) {
                        for(i = 0; i < 8; i++) {
                            calcChkSum = calcChkSum ^ a[i];
                        }

                        if(calcChkSum == a[8]) {
//                            cout<<"Checksum equal " << endl;
                            accel = a[0]/100.0;
                            brake[0] = a[1]/100.0;
                            brake[1] = a[2]/100.0;
                            brake[2] = a[3]/100.0;
                            brake[3] = a[4]/100.0;
                            angle = (int8_t)a[5]/50.0;
                            gear = (int8_t)a[6];
                            clutch = a[7]/100.0;     
                       }
                     // cout<<"Checksum Not equal " << endl;
                    }
                }
            }
        }
    }

    #endif

    //cout<<"End of Serial " << endl;
	int tmp = (int)car->_speed_x;
    b[0] = 0xAA;
    b[1] = 0xCC;
    b[2] =car->_speed_x;

	//cout<<(int)b[2]<<endl;
    uint16_t rpm = car->_enginerpm;
    b[3] = (rpm & 0xFF00) >> 8;
    b[4] = rpm & 0xFF;
 
    b[5] = car->_wheelSpinVel(FRNT_LFT) * car->_wheelRadius(FRNT_LFT);
    b[6] = car->_wheelSpinVel(FRNT_RGT) * car->_wheelRadius(FRNT_RGT);
    b[7] = car->_wheelSpinVel(REAR_LFT) * car->_wheelRadius(REAR_LFT);
    b[8] = car->_wheelSpinVel(REAR_RGT) * car->_wheelRadius(REAR_RGT);

    b[9] = car->_yaw_rate * 10;
    float disttomiddle;
    float randbase = 0;
    if(ocar[currentTrackID].getCarPtr()->_trkPos.toMiddle - ocar[0].getCarPtr()->_trkPos.toMiddle >= 0) disttomiddle = ocar[currentTrackID].getCarPtr()->_trkPos.toMiddle - ocar[0].getCarPtr()->_trkPos.toMiddle;
    else disttomiddle = ocar[0].getCarPtr()->_trkPos.toMiddle - ocar[currentTrackID].getCarPtr()->_trkPos.toMiddle;
    b[10] = disttomiddle *10;
    b[11] = ocar[currentTrackID].getCarPtr()->_speed_x;
    uint16_t dist;
    float distd;
    float angle1 = 0,k1,k2,delta_d1,delta_d2;
    if(ocar[currentTrackID].getCurrentSegId()>ocar[0].getCurrentSegId())	dist = ocar[currentTrackID].getCurrentSegId()-ocar[0].getCurrentSegId();
    else	dist = 1908 - ocar[0].getCurrentSegId() + ocar[currentTrackID].getCurrentSegId();
    if(dist>=400)	b[14] = 0;
    else	b[14] = IsTrackable(ocar[0].getCurrentSegId(),ocar[currentTrackID].getCurrentSegId());
    if(b[14])	{
	
      disttomiddle = ocar[currentTrackID].getCarPtr()->_trkPos.toMiddle - car->_trkPos.toMiddle;				//real Radar Simulator
      angle1 = RtTrackSideTgAngleL(&(car->_trkPos))-car->_yaw;
      NORM_PI_PI(angle1);
      angle1 = atan(disttomiddle/dist) - angle1;
      angle1 = angle1/3.141593*180;
      if (angle1<0) angle1 = 0 - angle1;
      distd = sqrt(float(disttomiddle*disttomiddle + dist*dist));
      dist = (int) distd;
      if(angle1>35){b[14] = 0;b[15] = 0;}
      else{
	k1 = angle1 / 35;
	k2 = distd / 300;
	delta_d1 = 0;
	delta_d2 = 0;
	randbase = RandomG();
	delta_d1 = k1*float(randbase - 5000)/5000.0;
	randbase = RandomG();
	delta_d2 = k2*float(randbase - 5000)/5000.0;
//	delta_d1 = k1*float(rand() % 10000 - 5000) / 5000.0;
//	delta_d2 = k2*float(rand() % 10000 - 5000) / 5000.0;	
	b[11] = b[11] + delta_d1 + delta_d2;
	dist = int(distd + delta_d1 +delta_d2);
	b[15] = int((delta_d1+delta_d2)*50);
      }

	//std::cout<<"angle = "<<angle1<<" delta_d = "<<delta_d1+delta_d2<<" dist = "<<dist<<std::endl;
    }
    //std::cout<<"speed = "<<ocar[currentTrackID].getCarPtr()->_speed_x<<std::endl;
    else {
	b[15] =0;
    }
    b[12] = (dist & 0xFF00) >> 8;
    b[13] = (dist & 0xFF);
    uint16_t test1 = b[13]+(b[12]<<8);
    
    b[16] = currentTrackID;
    b[17] = char(angle1);
    float angleTemp = RtTrackSideTgAngleL(&(car->_trkPos))-car->_yaw;
    NORM_PI_PI(angleTemp);
    b[18] = angleTemp/3.141593*180;
    calcChkSum = 0;
 
    for(i = 0; i <= 18; i++){   // Changed from 10 to 18 for second car // Changed from 9 to 10
        calcChkSum = calcChkSum ^ b[i];
    }
    
    b[19] = calcChkSum;  // changed from 11 to 19 for new distance method // Changed - was b[10] earlier // 


    serialPortWrite(serialfd, b, 20);  // Changed from 1 for distance // Changed size from 11 to 12
}


/* Start a new race. */
static void newrace(int index, tCarElt* car_local, tSituation *s)
{
	#if 1
//   if(index == 0) 
    {   // New code

    serialfd = serialPortOpen("/dev/ttyUSB0", 115200);
    car = car_local;
    timerid = timerInit(serialData, 2000000);
    serialDataStopped = false;
    enableTimerSignal();
   }
   #endif
	if (ocar != NULL) delete [] ocar;
	ocar = new OtherCar[s->_ncars];
	for (int i = 0; i < s->_ncars; i++) {
		ocar[i].init(myTrackDesc, s->cars[i], s);
	}

	if (mycar[index-1] != NULL) delete mycar[index-1];
	mycar[index-1] = new MyCar(myTrackDesc, car, s);

	currenttime = s->currentTime;

}

/* Drive during race. */
static void drive(int index, tCarElt* car, tSituation *s)
{

      memset((void *)&car->ctrl, 0, sizeof(tCarCtrl));

    //update other cars' information
	if (currenttime != s->currentTime) {
		currenttime = s->currentTime;
		for (int i = 0; i < s->_ncars; i++) {
			ocar[i].update();
			//if(ocar[i].getCarPtr() == car)	std::cout<<"MycarPtr = "<<i<<std::endl;
			//std::cout<<"Ocar.speed = "<<ocar[i].getSpeed()<<" Ocar.currentSegId = "<<ocar[i].getCurrentSegId()<<" Ocar.currentCursor = ["<<ocar[i].getCurrentPos()[0].x<<" "<<ocar[i].getCurrentPos()[0].y<<"] "<<ocar[i].getCarPtr()->_trkPos.toMiddle<<std::endl;
		}
		int num = 3;
		int temp = currentTrackID;
		currentTrackID = GetTrackableCarID(num);
		if(currentTrackID != temp){
			std::cout<<"ID changed! curretID = "<<currentTrackID<<std::endl;
		}
	}
     // set the values
     car->_steerCmd = angle;
     car->_gearCmd = gear;
     car->_accelCmd = accel;
     car->_individualBrakes = true;
     car->_individualBrakeCmd[0] = brake[0];
     car->_individualBrakeCmd[1] = brake[1];
     car->_individualBrakeCmd[2] = brake[2];
     car->_individualBrakeCmd[3] = brake[3];
     car->_brakeCmd = (brake[0] + brake[1] + brake[2] + brake[3])/4.0; // Just for display in TORCS
     car->_clutchCmd = clutch;

	//from Berniw
	tdble brake;
	tdble b1;							/* brake value in case we are to fast HERE and NOW */
	tdble b2;							/* brake value for some brake point in front of us */
	tdble b3;							/* brake value for control (avoid loosing control) */
	tdble b4;							/* brake value for avoiding high angle of attack */


   
   
}

/* End of the current race */
static void endrace(int index, tCarElt *car, tSituation *s)
{
    printf("endRace\n");
}

/* Called before the module is unloaded */
static void shutdown(int index)
{
	int i = index - 1;
	if (mycar[i] != NULL) {
		delete mycar[i];
		mycar[i] = NULL;
		//free(botdesc[i]);
		//free(botname[i]);
	}
	if (myTrackDesc != NULL) {
		delete myTrackDesc;
		myTrackDesc = NULL;
	}
	if (ocar != NULL) {
		delete [] ocar;
		ocar = NULL;
	}
    signalStopSendData = true;
    while(!serialDataStopped);
    disableTimerSignal();
    timerEnd(timerid);
    serialPortClose(serialfd);
 

}
