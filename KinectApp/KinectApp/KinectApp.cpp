/********************************************************************
 * ECE4007 Senior Design
 * KinectApp.cpp : Code for the Kinect to do skeletal tracking and 
 *				   transform the data to joint angles
 * Authot: Peihsun (Ben) Yeh
 * 
 * Basic program setup is provided by this tutorial:
 * http://mzubair.com/getting-started-building-your-first-kinect-app-with-c-in-visual-studio/
 *******************************************************************/

#include "stdafx.h"
#include <iostream>
#include <Windows.h>
#include <NuiApi.h>
#include <time.h>

// definitions for various body parts
// and for making the code easier to read
#define PI 3.14159
#define body myFrame.SkeletonData[0]
#define HEAD body.SkeletonPositions[NUI_SKELETON_POSITION_HEAD]
#define LSHOULDER body.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT]
#define RSHOULDER body.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT]
#define CSHOULDER body.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER]
#define ELBOW body.SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT]
#define WRIST body.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT]
#define HAND body.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT]
#define HIP body.SkeletonPositions[NUI_SKELETON_POSITION_HIP_CENTER]

using namespace std;

// function prototypes
FLOAT getAngle(FLOAT x1, FLOAT y1, FLOAT z1, FLOAT x2, FLOAT y2, FLOAT z2);
void delay_ms(float s); // kind of hackish, but works well enough since program's not doing much else

// misc variables...
FLOAT temp, temp2, angle;

int _tmain(int argc, _TCHAR* argv[])
{
	NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON);
	NUI_SKELETON_FRAME myFrame;

	/***** POSSIBLE SOURCE OF ERROR: COM PORT MISMATCH *****/
	// open a com port to send data to the Arduino
	HANDLE porthandle = CreateFile(L"\\\\.\\COM9", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	DCB dcbSerialParams;
  GetCommState(porthandle,&dcbSerialParams);
  dcbSerialParams.BaudRate = CBR_9600;
  dcbSerialParams.ByteSize = 8;
  dcbSerialParams.Parity = NOPARITY;
  dcbSerialParams.StopBits = ONESTOPBIT;
  SetCommState(porthandle, &dcbSerialParams);
	DWORD nBytesWritten;

	int historyLength = 4, historyInd = 0;
	bool inMotion[] = {0, 0, 0};
	//BYTE baseBuffer[] =		{'B', 90}; 						
	//BYTE shoulderBuffer[] = {'S', 90};					
	//BYTE elbowBuffer[] =	{'E', 135};					
	BYTE lastPositions[] =	{0, 0, 0};
	BYTE serialBuffer[] =	{0, 0, 0};										// buffer to be sent to Arduino (0 = base, 1 = shoulder, 2 = elbow)
	FLOAT baseHistoryBuffer[] =		{90, 90, 90, 90};			// moving window of base angles
	FLOAT shoulderHistoryBuffer[] = {90, 90, 90, 90};		// moving window of shoulder angles
	FLOAT elbowHistoryBuffer[] =	{135, 135, 135, 135};	// moving window of elbow angles

	while (1) 
	{
		NuiSkeletonGetNextFrame(0, &myFrame); //Get a frame and stuff it into myframe
		if (myFrame.SkeletonData[0].eTrackingState == NUI_SKELETON_TRACKED){ 
			// may or may not help
			NuiTransformSmooth(&myFrame, 0);

			// printing coordinates
			printf("Format: (x, y, z)\n");
			printf("Head:       (%f, %f, %f)\n", HEAD.x, HEAD.y, HEAD.z);
			printf("L shoulder: (%f, %f, %f)\n", LSHOULDER.x, LSHOULDER.y, LSHOULDER.z);
			printf("R shoulder: (%f, %f, %f)\n", RSHOULDER.x, RSHOULDER.y, RSHOULDER.z);
			printf("R elbow:    (%f, %f, %f)\n", ELBOW.x, ELBOW.y, ELBOW.z);
			printf("Wrist:      (%f, %f, %f)\n\n", WRIST.x, WRIST.y, WRIST.z);

			/***** Base *****/
			angle = getAngle(LSHOULDER.x - RSHOULDER.x, LSHOULDER.y - RSHOULDER.y, LSHOULDER.z - RSHOULDER.z,			
							 ELBOW.x	 - RSHOULDER.x, ELBOW.y		- RSHOULDER.y, ELBOW.z	   - RSHOULDER.z);
			baseHistoryBuffer[historyInd] = angle;
			temp = 0;
			for(int i = 0; i < historyLength; i++){
				temp+=baseHistoryBuffer[i];
			}
			temp/=historyLength;
			serialBuffer[0] = 180 - (char)(temp) + 10; // map calculated angles to servo angles and put data into buffer to be sent

			if(temp != temp) cout << "Base: bad angle\n";
			else cout << "Base angle: " << (int)serialBuffer[0] << endl;

			/***** Shoulder *****/
			angle = getAngle(HIP.x	 - CSHOULDER.x, HIP.y	- CSHOULDER.y, HIP.z	- CSHOULDER.z,			
							 ELBOW.x - RSHOULDER.x, ELBOW.y	- RSHOULDER.y, ELBOW.z	- RSHOULDER.z);
			shoulderHistoryBuffer[historyInd] = angle;
			temp = 0;
			for(int i = 0; i < historyLength; i++){
				temp+=shoulderHistoryBuffer[i];
			}
			temp/=historyLength;
			serialBuffer[1] = (char)temp + 90; // map and put data into buffer to be sent

			if(temp != temp) cout << "Shoulder: bad angle\n";
			else cout << "Shoulder angle: " << (int)serialBuffer[1] << endl;
			
			/***** Elbow *****/
			angle = getAngle(RSHOULDER.x - ELBOW.x, RSHOULDER.y - ELBOW.y, RSHOULDER.z - ELBOW.z,
                       WRIST.x     - ELBOW.x, WRIST.y     - ELBOW.y, WRIST.z     - ELBOW.z);
			elbowHistoryBuffer[historyInd] = angle;
			temp = 0;
			for(int i = 0; i < historyLength; i++){
				temp+=elbowHistoryBuffer[i];
			}
			temp/=historyLength;
			serialBuffer[2] = (char)temp - 25; // map and put data into buffer to be sent

			if(temp != temp) cout << "Elbow: bad angle\n";
			else cout << "Elbow angle: " << (int)serialBuffer[2] << endl;

			historyInd = (historyInd+1)%historyLength;

			if(temp != temp) { // check for NaN
			} else {
				cout << "sending coordinates...\n";
				WriteFile(porthandle, serialBuffer, sizeof(serialBuffer), &nBytesWritten, NULL);
				delay_ms(10);
			}
		} // if (myFrame.SkeletonData[0]...)
		else 
		{
			cout << "Skeleton not tracked...\nplease move back into frame or restart program\n";
		}
		system("cls");
	}
	NuiShutdown();
	return 0;
}

// function definitions

/**************************************************************************
 * getAngles
 * gets the angle between two vectors using the formula
 *		cos(theta) = (A dot B)/(|A||B|)
 *************************************************************************/
FLOAT getAngle(FLOAT x1, FLOAT y1, FLOAT z1, FLOAT x2, FLOAT y2, FLOAT z2){
	FLOAT num = (x1*x2)+(y1*y2)+(z1*z2);
	FLOAT len1 = sqrt(x1*x1 + y1*y1 + z1*z1);
	FLOAT len2 = sqrt(x2*x2 + y2*y2 + z2*z2);
	//cout << num << ", " << len1 << ", " << len2 << endl;
	return (180.0/PI)*acos(num/(len1*len2));
}

/************************************************************************
 * void delay_ms(float s)
 * delays operation for s milliseconds
 ************************************************************************/
void delay_ms(float s){
	clock_t t1 = clock();
	while(((float)(clock()-t1)*1000)/CLOCKS_PER_SEC < s) {}
}
