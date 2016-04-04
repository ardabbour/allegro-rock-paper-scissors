//
// 20141209: kcchang: changed window version to linux 

// myAllegroHand.cpp : Defines the entry point for the console application.
//
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>  //_getch
#include <string.h>
#include <pthread.h>
#include <math.h>
#include "canAPI.h"
#include "rDeviceAllegroHandCANDef.h"
#include "RockScissorsPaper.h"
#include <BHand/BHand.h> 

static struct termios old; 
static struct termios newa;

void initTermios(int echo) 
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  newa = old; /* make new settings same as old settings */
  newa.c_lflag &= ~ICANON; /* disable buffered i/o */
  newa.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &newa); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo) 
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

char getch(void) 
{
  return getch_(0);
}

typedef char    TCHAR;
#define _T(X)   X
#define _tcsicmp(x, y)   strcmp(x, y)

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////
// for CAN communication
const double delT = 0.003;
int CAN_Ch = 0;
bool ioThreadRun = false;
pthread_t        hThread;
int recvNum = 0;
int sendNum = 0;
double statTime = -1.0;
AllegroHand_DeviceMemory_t vars;

double curTime = 0.0;

/////////////////////////////////////////////////////////////////////////////////////////
// for BHand library
BHand* pBHand = NULL;
double q[MAX_DOF];
double q_des[MAX_DOF];
double tau_des[MAX_DOF];
double cur_des[MAX_DOF];

// USER HAND CONFIGURATION
const bool	RIGHT_HAND = true;
const int	HAND_VERSION = 3;

const double tau_cov_const_v3 = 1200.0; // 1200.0 for SAH030xxxxx
const double tau_cov_const_v2 = 800.0; // 800.0 for SAH20xxxxx

const double enc_dir[MAX_DOF] = { // SAH030xxxxx
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0
};
const double motor_dir[MAX_DOF] = { // SAH030xxxxx
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0
};
int enc_offset[MAX_DOF] = { // SAH030BR029
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0
};

/////////////////////////////////////////////////////////////////////////////////////////
// functions declarations
// char Getch();
void PrintInstruction();
void MainLoop();
bool OpenCAN();
void CloseCAN();
int GetCANChannelIndex(const TCHAR* cname);
bool CreateBHandAlgorithm();
void DestroyBHandAlgorithm();
void ComputeTorque();

/////////////////////////////////////////////////////////////////////////////////////////
// CAN communication thread
static void* ioThreadProc(void* inst)
{
  char id_des;
  char id_cmd;
  char id_src;
  int len;
  unsigned char data[8];
  unsigned char data_return = 0;
  int i;

  while (ioThreadRun)
    {
      /* wait for the event */
      while (0 == get_message(CAN_Ch, &id_cmd, &id_src, &id_des, &len, data, FALSE))
	{
	  switch (id_cmd)
	    {
	    case ID_CMD_QUERY_ID:
	      {
		/*printf(">CAN(%d): AllegroHand revision info: 0x%02x%02x\n", CAN_Ch, data[3], data[2]);
		printf("                      firmware info: 0x%02x%02x\n", data[5], data[4]);
		printf("                      hardware type: 0x%02x\n", data[7]);*/
	      }
	      break;

	    case ID_CMD_AHRS_POSE:
	      {
		/*printf(">CAN(%d): AHRS Roll : 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
		printf("               Pitch: 0x%02x%02x\n", data[2], data[3]);
		printf("               Yaw  : 0x%02x%02x\n", data[4], data[5]);*/
	      }
	      break;

	    case ID_CMD_AHRS_ACC:
	      {
		/*printf(">CAN(%d): AHRS Acc(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
		printf("               Acc(y): 0x%02x%02x\n", data[2], data[3]);
		printf("               Acc(z): 0x%02x%02x\n", data[4], data[5]);*/
	      }
	      break;

	    case ID_CMD_AHRS_GYRO:
	      {
		/*printf(">CAN(%d): AHRS Angular Vel(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
		printf("               Angular Vel(y): 0x%02x%02x\n", data[2], data[3]);
		printf("               Angular Vel(z): 0x%02x%02x\n", data[4], data[5]);*/
	      }
	      break;

	    case ID_CMD_AHRS_MAG:
	      {
		/*printf(">CAN(%d): AHRS Magnetic Field(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
		printf("               Magnetic Field(y): 0x%02x%02x\n", data[2], data[3]);
		printf("               Magnetic Field(z): 0x%02x%02x\n", data[4], data[5]);*/
	      }
	      break;

	    case ID_CMD_QUERY_CONTROL_DATA:
	      {
		if (id_src >= ID_DEVICE_SUB_01 && id_src <= ID_DEVICE_SUB_04)
		  {
		    vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 0] = (int)(data[0] | (data[1] << 8));
		    vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 1] = (int)(data[2] | (data[3] << 8));
		    vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 2] = (int)(data[4] | (data[5] << 8));
		    vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 3] = (int)(data[6] | (data[7] << 8));
		    data_return |= (0x01 << (id_src-ID_DEVICE_SUB_01));
		    recvNum++;
		  }
		if (data_return == (0x01 | 0x02 | 0x04 | 0x08))
		  {
		    // convert encoder count to joint angle
		    for (i=0; i<MAX_DOF; i++)
		      {
			q[i] = (double)(vars.enc_actual[i]*enc_dir[i]-32768-enc_offset[i])*(333.3/65536.0)*(3.141592/180.0);
		      }

		    // compute joint torque
		    ComputeTorque();

		    // convert desired torque to desired current and PWM count
		    for (i=0; i<MAX_DOF; i++)
		      {
			cur_des[i] = tau_des[i] * motor_dir[i];
			if (cur_des[i] > 1.0) cur_des[i] = 1.0;
			else if (cur_des[i] < -1.0) cur_des[i] = -1.0;
		      }

		    // send torques
		    for (int i=0; i<4;i++)
		      {
			// the index order for motors is different from that of encoders
			switch (HAND_VERSION)
			  {
			  case 1:
			  case 2:
			    vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+0]*tau_cov_const_v2);
			    vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+1]*tau_cov_const_v2);
			    vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+2]*tau_cov_const_v2);
			    vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+3]*tau_cov_const_v2);
			    break;

			  case 3:
			  default:
			    vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+0]*tau_cov_const_v3);
			    vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+1]*tau_cov_const_v3);
			    vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+2]*tau_cov_const_v3);
			    vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+3]*tau_cov_const_v3);
			    break;
			  }
			write_current(CAN_Ch, i, &vars.pwm_demand[4*i]);
			usleep(5);
		      }
		    sendNum++;
		    curTime += delT;

		    data_return = 0;
		  }
	      }
	      break;
	    }
	}
    }

  return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Application main-loop. It handles the commands from rPanelManipulator and keyboard events

void MainLoop() {
  bool bRun = true;
  while (bRun) {
    	int choice = 0;	
	printf("Getting ready...\n");
	sleep(1);
	pBHand->SetMotionType(eMotionType_READY);
	printf("I'm ready!\n");
	sleep(1);
	int go = getch();
	if (go == 'g') {
	  FILE *input;
	  input = fopen("/home/abood/sketchbook/RockPaperScissorRecognition/gesture.txt", "r");
	  if (input) {
	    choice = getc(input);
	    fclose(input);
          }
	  if (choice == '1' || choice == '2' || choice == '3'){
		srand ( time(NULL) );
	    int c = round(1 + (rand() % 2));
      	    switch (c)
        	{
		case 1:
		  MotionRock();
		  if (choice == '1'){
			printf("You played Rock; Allegro Hand played Rock.\n");
			printf("You drew with Allegro Hand! Let's try that again!\n");
		  }
		  if (choice == '2'){
			printf("You played Paper; Allegro Hand played Rock.\n");
			printf("Congratulations! You beat Allegro Hand! Let's try that again!\n");
		  }
		  if (choice == '3'){
			printf("You played Scissors; Allegro Hand played Rock.\n");
			printf("Too bad! You lost against Allegro Hand! Let's try that again!\n");
		  }
		  printf("Restarting in 3...\n");
		  sleep(1);
		  printf("Restarting in 2...\n");
		  sleep(1);
		  printf("Restarting in 1...\n");
		  sleep(1);
		  break;
	
		case 2:
		  MotionPaper();
		  if (choice == '1'){
			printf("You played Rock; Allegro Hand played Paper.\n");
			printf("Too bad! You lost against Allegro Hand! Let's try that again!\n");
		  }
		  if (choice == '2'){
			printf("You played Paper; Allegro Hand played Paper.\n");
			printf("You drew with Allegro Hand! Let's try that again! Let's try that again!\n");
		  }
		  if (choice == '3'){
			printf("You played Scissors; Allegro Hand played Paper.\n");
			printf("Congratulations! You beat Allegro Hand! Let's try that again!\n");
		  }
		  printf("Restarting in 3...\n");
		  sleep(1);
		  printf("Restarting in 2...\n");
		  sleep(1);
		  printf("Restarting in 1...\n");
		  sleep(1);
		  break;
	
		case 3:
		  MotionScissors();
		  if (choice == '1'){
			printf("You played Rock; Allegro Hand played Scissors.\n");
			printf("Congratulations! You beat Allegro Hand! Let's try that again!\n");
		  }
		  if (choice == '2'){
			printf("You played Paper; Allegro Hand played Scissors.\n");
			printf("Too bad! You lost against Allegro Hand! Let's try that again!\n");
		  }
		  if (choice == '3'){
			printf("You played Scissors; Allegro Hand played Scissors.\n");
			printf("You drew with Allegro Hand! Let's try that again!\n");
		  }
		  printf("Restarting in 3...\n");
		  sleep(1);
		  printf("Restarting in 2...\n");
		  sleep(1);
		  printf("Restarting in 1...\n");
		  sleep(1);
		  break;
	        }
	}
    }
}
}

/////////////////////////////////////////////////////////////////////////////////////////
// Compute control torque for each joint using BHand library
void ComputeTorque()
{
  if (!pBHand) return;
  pBHand->SetJointPosition(q); // tell BHand library the current joint positions
  pBHand->SetJointDesiredPosition(q_des);
  pBHand->UpdateControl(0);
  pBHand->GetJointTorque(tau_des);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Open a CAN data channel
bool OpenCAN()
{
#if defined(PEAKCAN)
  CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
#elif defined(IXXATCAN)
  CAN_Ch = 1;
#elif defined(SOFTINGCAN)
  CAN_Ch = 1;
#else
  CAN_Ch = 1;
#endif
  CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
  //printf(">CAN(%d): open\n", CAN_Ch);

  int ret = command_can_open(CAN_Ch);
  if(ret < 0)
    {
      //printf("ERROR command_canopen !!! \n");
      return false;
    }

  ioThreadRun = true;

  /* initialize condition variable */
  int ioThread_error = pthread_create(&hThread, NULL, ioThreadProc, 0);
  //printf(">CAN: starts listening CAN frames\n");

  //printf(">CAN: query system id\n");
  ret = command_can_query_id(CAN_Ch);
  if(ret < 0)
    {
      //printf("ERROR command_can_query_id !!! \n");
      command_can_close(CAN_Ch);
      return false;
    }

  //printf(">CAN: AHRS set\n");
  ret = command_can_AHRS_set(CAN_Ch, AHRS_RATE_100Hz, AHRS_MASK_POSE | AHRS_MASK_ACC);
  if(ret < 0)
    {
      //printf("ERROR command_can_AHRS_set !!! \n");
      command_can_close(CAN_Ch);
      return false;
    }

  //printf(">CAN: system init\n");
  ret = command_can_sys_init(CAN_Ch, 3/*msec*/);
  if(ret < 0)
    {
      //printf("ERROR command_can_sys_init !!! \n");
      command_can_close(CAN_Ch);
      return false;
    }

  //printf(">CAN: start periodic communication\n");
  ret = command_can_start(CAN_Ch);

  if(ret < 0)
    {
      //printf("ERROR command_can_start !!! \n");
      command_can_stop(CAN_Ch);
      command_can_close(CAN_Ch);
      return false;
    }

  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Close CAN data channel
void CloseCAN()
{
  //printf(">CAN: stop periodic communication\n");
  int ret = command_can_stop(CAN_Ch);
  if(ret < 0)
    {
    //  printf("ERROR command_can_stop !!! \n");
    }

  if (ioThreadRun)
    {
      //printf(">CAN: stoped listening CAN frames\n");
      ioThreadRun = false;
      int status;
      pthread_join(hThread, (void **)&status);
      hThread = 0;
    }

  //printf(">CAN(%d): close\n", CAN_Ch);
  ret = command_can_close(CAN_Ch);
  if(ret < 0) {}//printf("ERROR command_can_close !!! \n");
}

/////////////////////////////////////////////////////////////////////////////////////////
// Load and create grasping algorithm
bool CreateBHandAlgorithm()
{
  if (RIGHT_HAND)
    pBHand = bhCreateRightHand();
  else
    pBHand = bhCreateLeftHand();

  if (!pBHand) return false;
  pBHand->SetTimeInterval(delT);
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Destroy grasping algorithm
void DestroyBHandAlgorithm()
{
  if (pBHand)
    {
#ifndef _DEBUG
      delete pBHand;
#endif
      pBHand = NULL;
    }
}

////////////////////////////////////////////////////////////////////////////////////////
// Print program information and keyboard instructions
void PrintInstruction()
{
  printf("-------------------------------------------\n");
  printf("BAU Allegro Hand: ");
  if (RIGHT_HAND) printf("Right Hand, v%i.x\n", HAND_VERSION); else printf("Left Hand, v%i.x\n", HAND_VERSION);
  printf("Allegro Hand code orginally developed by Kyong-Sok Chang\n");
  printf("Robotic Rock-Paper-Scissors by A. Dabbour\n");
  printf("-------------------------------------------\n\n");
}
/////////////////////////////////////////////////////////////////////////////////////////
// Get channel index for Peak CAN interface
int GetCANChannelIndex(const TCHAR* cname)
{
  if (!cname) return 0;

  if (!_tcsicmp(cname, _T("0")) || !_tcsicmp(cname, _T("PCAN_NONEBUS")) || !_tcsicmp(cname, _T("NONEBUS")))
    return 0;
  else if (!_tcsicmp(cname, _T("1")) || !_tcsicmp(cname, _T("PCAN_ISABUS1")) || !_tcsicmp(cname, _T("ISABUS1")))
    return 1;
  else if (!_tcsicmp(cname, _T("2")) || !_tcsicmp(cname, _T("PCAN_ISABUS2")) || !_tcsicmp(cname, _T("ISABUS2")))
    return 2;
  else if (!_tcsicmp(cname, _T("3")) || !_tcsicmp(cname, _T("PCAN_ISABUS3")) || !_tcsicmp(cname, _T("ISABUS3")))
    return 3;
  else if (!_tcsicmp(cname, _T("4")) || !_tcsicmp(cname, _T("PCAN_ISABUS4")) || !_tcsicmp(cname, _T("ISABUS4")))
    return 4;
  else if (!_tcsicmp(cname, _T("5")) || !_tcsicmp(cname, _T("PCAN_ISABUS5")) || !_tcsicmp(cname, _T("ISABUS5")))
    return 5;
  else if (!_tcsicmp(cname, _T("7")) || !_tcsicmp(cname, _T("PCAN_ISABUS6")) || !_tcsicmp(cname, _T("ISABUS6")))
    return 6;
  else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS7")) || !_tcsicmp(cname, _T("ISABUS7")))
    return 7;
  else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS8")) || !_tcsicmp(cname, _T("ISABUS8")))
    return 8;
  else if (!_tcsicmp(cname, _T("9")) || !_tcsicmp(cname, _T("PCAN_DNGBUS1")) || !_tcsicmp(cname, _T("DNGBUS1")))
    return 9;
  else if (!_tcsicmp(cname, _T("10")) || !_tcsicmp(cname, _T("PCAN_PCIBUS1")) || !_tcsicmp(cname, _T("PCIBUS1")))
    return 10;
  else if (!_tcsicmp(cname, _T("11")) || !_tcsicmp(cname, _T("PCAN_PCIBUS2")) || !_tcsicmp(cname, _T("PCIBUS2")))
    return 11;
  else if (!_tcsicmp(cname, _T("12")) || !_tcsicmp(cname, _T("PCAN_PCIBUS3")) || !_tcsicmp(cname, _T("PCIBUS3")))
    return 12;
  else if (!_tcsicmp(cname, _T("13")) || !_tcsicmp(cname, _T("PCAN_PCIBUS4")) || !_tcsicmp(cname, _T("PCIBUS4")))
    return 13;
  else if (!_tcsicmp(cname, _T("14")) || !_tcsicmp(cname, _T("PCAN_PCIBUS5")) || !_tcsicmp(cname, _T("PCIBUS5")))
    return 14;
  else if (!_tcsicmp(cname, _T("15")) || !_tcsicmp(cname, _T("PCAN_PCIBUS6")) || !_tcsicmp(cname, _T("PCIBUS6")))
    return 15;
  else if (!_tcsicmp(cname, _T("16")) || !_tcsicmp(cname, _T("PCAN_PCIBUS7")) || !_tcsicmp(cname, _T("PCIBUS7")))
    return 16;
  else if (!_tcsicmp(cname, _T("17")) || !_tcsicmp(cname, _T("PCAN_PCIBUS8")) || !_tcsicmp(cname, _T("PCIBUS8")))
    return 17;
  else if (!_tcsicmp(cname, _T("18")) || !_tcsicmp(cname, _T("PCAN_USBBUS1")) || !_tcsicmp(cname, _T("USBBUS1")))
    return 18;
  else if (!_tcsicmp(cname, _T("19")) || !_tcsicmp(cname, _T("PCAN_USBBUS2")) || !_tcsicmp(cname, _T("USBBUS2")))
    return 19;
  else if (!_tcsicmp(cname, _T("20")) || !_tcsicmp(cname, _T("PCAN_USBBUS3")) || !_tcsicmp(cname, _T("USBBUS3")))
    return 20;
  else if (!_tcsicmp(cname, _T("21")) || !_tcsicmp(cname, _T("PCAN_USBBUS4")) || !_tcsicmp(cname, _T("USBBUS4")))
    return 21;
  else if (!_tcsicmp(cname, _T("22")) || !_tcsicmp(cname, _T("PCAN_USBBUS5")) || !_tcsicmp(cname, _T("USBBUS5")))
    return 22;
  else if (!_tcsicmp(cname, _T("23")) || !_tcsicmp(cname, _T("PCAN_USBBUS6")) || !_tcsicmp(cname, _T("USBBUS6")))
    return 23;
  else if (!_tcsicmp(cname, _T("24")) || !_tcsicmp(cname, _T("PCAN_USBBUS7")) || !_tcsicmp(cname, _T("USBBUS7")))
    return 24;
  else if (!_tcsicmp(cname, _T("25")) || !_tcsicmp(cname, _T("PCAN_USBBUS8")) || !_tcsicmp(cname, _T("USBBUS8")))
    return 25;
  else if (!_tcsicmp(cname, _T("26")) || !_tcsicmp(cname, _T("PCAN_PCCBUS1")) || !_tcsicmp(cname, _T("PCCBUS1")))
    return 26;
  else if (!_tcsicmp(cname, _T("27")) || !_tcsicmp(cname, _T("PCAN_PCCBUS2")) || !_tcsicmp(cname, _T("PCCBUS2")))
    return 271;
  else
    return 0;
}


/////////////////////////////////////////////////////////////////////////////////////////
// Program main
int main(int argc, TCHAR* argv[])
{
	// offsets: load from file which given in cmdline arg #1 (#0 is program itself)
	if(argc > 1)
	{
		FILE *fp = fopen(argv[1],"r");
		for(int i=0;i<MAX_DOF;i++)
			fscanf(fp,"%d",&(enc_offset[i]));
		fclose(fp);
	}
  PrintInstruction();

  memset(&vars, 0, sizeof(vars));
  memset(q, 0, sizeof(q));
  memset(q_des, 0, sizeof(q_des));
  memset(tau_des, 0, sizeof(tau_des));
  memset(cur_des, 0, sizeof(cur_des));
  curTime = 0.0;

  if (CreateBHandAlgorithm() && OpenCAN()){
    MainLoop();
  }
  CloseCAN();
  DestroyBHandAlgorithm();

  return 0;
}
