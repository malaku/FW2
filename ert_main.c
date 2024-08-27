/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ert_main.c
 *
 * Code generated for Simulink model 'FW2'.
 *
 * Model version                  : 1.66
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Thu Nov 30 00:42:37 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include <stdio.h>
#include <stdlib.h>
#include "FW2.h"
#include "rtwtypes.h"
#include "limits.h"
#include "linuxinitialize.h"
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h> // Add this line for write() and close()
#include <linux/can.h>
#include <linux/can/raw.h>
#include <arpa/inet.h>

#define UNUSED(x)                      x = x
#define NAMELEN                        16
#define CAN_INTERFACE "can1"
#define SERVER_PORT 8080
/* Function prototype declaration*/
int openCANSocket(void);
void closeCANSocket(int socketDescriptor);
void sendCANMessage(int socketDescriptor, uint32_t messageID, float value);
void receiveCANMessage(int socketDescriptor, int targetID, float *tempvar);
//void receiveFirstCANMessage(int socketDescriptor, int *receivedCANID);
int det=0;

void exitFcn(int sig);
void *terminateTask(void *arg);
void *baseRateTask(void *arg);
void *subrateTask(void *arg);
volatile boolean_T stopRequested = false;
volatile boolean_T runModel = true;
sem_t stopSem;
sem_t baserateTaskSem;
pthread_t schedulerThread;
pthread_t baseRateThread;
void *threadJoinStatus;
int switchvar=1;
int terminatingmodel = 0;
int openEthernetSocket() {
    int sockfd;
    struct sockaddr_in serverAddr;

    // Create socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    // Set up server address
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT); // Example port number
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Example IP address

    // Connect to the server
    if (connect(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
        perror("connect");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    return sockfd;
}

void closeEthernetSocket(int socketDescriptor) {
    close(socketDescriptor);
}

void sendEthernetMessage(int socketDescriptor, const char *message) {
    if (send(socketDescriptor, message, strlen(message), 0) == -1) {
        perror("send");
    }
}

void receiveEthernetMessage(int socketDescriptor, char *buffer, size_t bufferSize) {
    ssize_t bytesRead = recv(socketDescriptor, buffer, bufferSize - 1, 0);
    if (bytesRead == -1) {
        perror("recv");
    } else {
        buffer[bytesRead] = '\0'; // Null-terminate the received data
        printf("Received Ethernet message: %s\n", buffer);
    }
}

void processValues(float desiredfw21, float desiredfw22, float desiredfw23, float desiredfw24, int socketDescriptor) {
    if (desiredfw21 == desiredfw22 && desiredfw22 == desiredfw23 && desiredfw23 == desiredfw24) {
        rtU.desiredfw2 = desiredfw21; // Perform the desired action here
sendCANMessage(socketDescriptor, 0x222, 0);
    } else if ((desiredfw21 == desiredfw22 && desiredfw22 == desiredfw23) ||
               (desiredfw22 == desiredfw23 && desiredfw23 == desiredfw24) ||
               (desiredfw23 == desiredfw24 && desiredfw24 == desiredfw21)) {
        if (desiredfw21 == desiredfw22) {
            rtU.desiredfw2 = desiredfw21;
sendCANMessage(socketDescriptor, 0x222, 0);
        } else if (desiredfw22 == desiredfw23) {
            rtU.desiredfw2 = desiredfw22;
sendCANMessage(socketDescriptor, 0x222, 0);
        } else {
            rtU.desiredfw2 = desiredfw23;
sendCANMessage(socketDescriptor, 0x222, 0);

        }
    } else {
                    sendCANMessage(socketDescriptor, 0x222, 1);
            // Set wasMessageSent to true to avoid sending the message repeatedly
        return;
    }
}

void *baseRateTask(void *arg)
{
  runModel = (rtmGetErrorStatus(rtM) == (NULL)) && !rtmGetStopRequested(rtM);
int socketDescriptor = openCANSocket();
int socketDescriptorEth = openEthernetSocket();
int receivedCANID;

int targetMessageID1 = 0x522; // Desired FW1
int targetMessageID2 = 0x622; // Desired FW1
int targetMessageID3 = 0x722; // Desired FW1
int targetMessageID4 = 0x7F2; // Desired FW1 
int targetMessageID5 = 0x011; // Fx
int targetMessageID6 = 0x022; // Fy
uint32_t messageID = 0x201;
uint8_t receivedData[4]; // Buffer to store received data

float Fx;
float Fy;
float desiredfw21;
float desiredfw22;
float desiredfw23;
float desiredfw24;
float ID;	
if (switchvar==1){
	while (runModel) {
	sem_wait(&baserateTaskSem);

		if (det==0){
			receiveCANMessage(socketDescriptor,0x555, &ID);
			if (ID==0){
				receiveCANMessage(socketDescriptor,targetMessageID5, &Fx);
				receiveCANMessage(socketDescriptor,targetMessageID6, &Fy);
				receiveCANMessage(socketDescriptor,targetMessageID1, &desiredfw21);
				receiveCANMessage(socketDescriptor,targetMessageID2, &desiredfw22);
				receiveCANMessage(socketDescriptor,targetMessageID3, &desiredfw23);
				receiveCANMessage(socketDescriptor,targetMessageID4, &desiredfw24);
				rtU.Fx = Fx;
				rtU.Fy=Fy;
				usleep(300);
				processValues(desiredfw21, desiredfw22, desiredfw23, desiredfw24, socketDescriptor);
				//usleep(100);
				FW2_step();
				float actualfw2_value = rtY.Actualfw2; // Assuming rtY.Actualfw2 is your float variable
				float fw2= rtY.Actualfw2;
				sendCANMessage(socketDescriptor, messageID, fw2);
			}else{
				det=1;
			}
		} else {
			printf("Waiting for Ethernet\n");
float desiredfw2;
			receiveCANMessage(socketDescriptor,0x053, &desiredfw2);
			rtU.desiredfw2=desiredfw2/16;
usleep(1000);
			FW2_step();
float fw2= rtY.Actualfw2;
usleep(500);
			sendCANMessage(socketDescriptor, messageID, fw2);

		}



	stopRequested = !((rtmGetErrorStatus(rtM) == (NULL)) && !rtmGetStopRequested
                     (rtM));
	runModel = !stopRequested;
	}
closeCANSocket(socketDescriptor);
closeEthernetSocket(socketDescriptorEth);

  runModel = 0;
  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;
}else{
while (runModel) {
    sem_wait(&baserateTaskSem);
receiveCANMessage(socketDescriptor,targetMessageID5, &Fx);
		receiveCANMessage(socketDescriptor,targetMessageID6, &Fy);
		receiveCANMessage(socketDescriptor,targetMessageID1, &desiredfw21);
		receiveCANMessage(socketDescriptor,targetMessageID2, &desiredfw22);
		receiveCANMessage(socketDescriptor,targetMessageID3, &desiredfw23);
		receiveCANMessage(socketDescriptor,targetMessageID4, &desiredfw24);
		rtU.Fx = Fx;
		rtU.Fy=Fy;
		usleep(300);
		rtU.desiredfw2 = desiredfw23;
		FW2_step();
		float actualfw2_value = rtY.Actualfw2; // Assuming rtY.Actualfw2 is your float variable
		float fw2= rtY.Actualfw2;
		sendCANMessage(socketDescriptor, messageID, fw2);
    stopRequested = !((rtmGetErrorStatus(rtM) == (NULL)) && !rtmGetStopRequested
                      (rtM));
    runModel = !stopRequested;
  }
}
closeCANSocket(socketDescriptor);
closeEthernetSocket(socketDescriptorEth);

  runModel = 0;
  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;

}
void exitFcn(int sig)
{
  UNUSED(sig);
  rtmSetErrorStatus(rtM, "stopping the model");
}

void *terminateTask(void *arg)
{
  UNUSED(arg);
  terminatingmodel = 1;

  {
    runModel = 0;
  }

  sem_post(&stopSem);
  return NULL;
}

int main(int argc, char **argv)
{
  rtmSetErrorStatus(rtM, 0);

  /* Initialize model */
  FW2_initialize();

  /* Call RTOS Initialization function */
  myRTOSInit(0.01, 0);

  /* Wait for stop semaphore */
  sem_wait(&stopSem);

#if (MW_NUMBER_TIMER_DRIVEN_TASKS > 0)

  {
    int i;
    for (i=0; i < MW_NUMBER_TIMER_DRIVEN_TASKS; i++) {
      CHECK_STATUS(sem_destroy(&timerTaskSem[i]), 0, "sem_destroy");
    }
  }

#endif

  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
