
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>         /* for read, write, close */
#include <sys/ioctl.h>      /* for ioctl */
#include <sys/types.h>      /* for open */
#include <sys/stat.h>       /* for open */
#include <fcntl.h>          /* for open */

#include <signal.h>

//#include "PanTilt.h"         
#include <iostream>
using namespace std;
#include "urbtc.h"          /* Linux specific part */
#include "urobotc.h"        /* OS independent part */
static char *pdev = {"/dev/urbtc0"};
//static char *tdev = {"/dev/urbtc3"};
#include <SDL/SDL.h>
SDL_Thread *th1;

long int pan_angle;
int person_position_x;

struct ccmd pcmd;
struct uin pbuf;
struct uout pobuf;
int pfd;
int person_old=160;
int person=160;
int p_velocity=32768;
int dv;
int Kp=50;
int Kt=50;

double pan_degree=0;
void pantilt_start();

void pantilt_init(){

	fprintf(stderr, "init\n");
    if ((pfd = open(pdev, O_RDWR)) == -1) 
	fprintf(stderr, "%s: Open error\n", pdev);
    if (ioctl(pfd, URBTC_CONTINUOUS_READ) < 0)
	fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
    if (read(pfd, &pbuf, sizeof(pbuf)) != sizeof(pbuf)) 
	fprintf(stderr, "Read size mismatch.\n");

    pcmd.retval = 0 /* RETURN_VAL */;
    pcmd.setoffset  = CH0 | CH1 | CH2 | CH3;
    pcmd.setcounter = CH0 | CH1 | CH2 | CH3;
    pcmd.resetint   = CH0 | CH1 | CH2 | CH3;
    pcmd.selin = SET_SELECT | SET_CH2_HIN; /* AD in:ch0,ch1    ENC in:ch2,ch3*/
    pcmd.dout = 0;
    pcmd.selout = SET_SELECT | CH2;//モータへの出力ポート
    pcmd.offset[0] = pcmd.offset[1] = pcmd.offset[2] = pcmd.offset[3] = 0x7fff;

    pcmd.counter[0] = pcmd.counter[1] = pcmd.counter[2] = pcmd.counter[3] = 0;
    pcmd.posneg = SET_POSNEG | CH0| CH1 | CH2 | CH3; /*POS PWM out*/
    pcmd.breaks = SET_BREAKS | CH0 | CH1 | CH2 | CH3; /*No Brake*/
    pcmd.magicno=0x00;
    pcmd.wrrom=0;


    if (ioctl(pfd, URBTC_COUNTER_SET) < 0)
	fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
    if (write(pfd, &pcmd, sizeof(pcmd)) < 0)
	fprintf(stderr, "pcmd write error\n");

    pcmd.setcounter = 0;
    p_velocity = 32768;

    if (write(pfd, &pcmd, sizeof(pcmd)) < 0)
	fprintf(stderr, "pcmd write error\n");
    if (ioctl(pfd, URBTC_DESIRE_SET) < 0)
	fprintf(stderr, "ioctl: URBTC_DESIRE_SET error\n");
    if (ioctl(pfd, URBTC_BUFREAD) < 0)
	fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");  
    if (ioctl(pfd, URBTC_CONTINUOUS_READ) < 0)
	fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");

    read(pfd,&pbuf,sizeof(pbuf));

}


int pantilt_end(){
    p_velocity=32768;
    pcmd.offset[2]=p_velocity;
    write(pfd, &pcmd, sizeof(pcmd));
    close(pfd);

}




int pantilt(){
    //ここから初期化
    int i=0;
    SDL_Joystick *joystick;
    int done=0;
    int axes;

  
    if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_JOYSTICK) < 0){
	return -1;
    }
    if(0 < SDL_NumJoysticks()){
	SDL_JoystickEventState(SDL_ENABLE);
	joystick = SDL_JoystickOpen(0);

	printf("JoystickName=%s\n", SDL_JoystickName(0));
	axes = SDL_JoystickNumAxes(joystick);
	printf("JoystickNumAxes=%d\n", axes);

	for (i=0; i < axes; i++){
	    printf("JoystickGetAxis(%d)=%d\n", i, (int)SDL_JoystickGetAxis(joystick, i));
	}
    }

    SDL_Event event;

    int NeckSpeed=0;
    //ここからループ
    //    qDebug("PanTilt-connect");
    int error_flg=0;
    while(!done){

	if(SDL_PollEvent(&event)){
	    //   printf("debug\n");
	    switch(event.type){
	    case SDL_QUIT:
		printf("quit\n");
	
		done = 1;

		read(pfd,&pbuf,sizeof(pbuf));
		//入力するエンコーダのチャンネル指定（scoutと違う）
		pan_angle=pbuf.ct[1];


		while(!((pan_angle<10)&&(pan_angle>=0))||((pan_angle>-10)&&(pan_angle<=0))){
		    error_flg++;
		    if(error_flg>10000) {
			printf("battery error");
			return -1;
		    }
		    if (ioctl(pfd, URBTC_CONTINUOUS_READ) < 0){
			fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
			break;
		    }

		    read(pfd,&pbuf,sizeof(pbuf));
		    //入力するエンコーダのチャンネル指定（scoutと違う）
		    pan_angle=pbuf.ct[1];

		    printf("%d\n",pan_angle);

		    if(pan_angle>=0) {
			p_velocity=32768+1000;	     
		    }

		    else if(pan_angle<0) {
			p_velocity=32768-1000;	     
		    }

		    pcmd.offset[2]=p_velocity;

		    if (ioctl(pfd, URBTC_COUNTER_SET) < 0){
			fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
			break;
		    }

		    if (write(pfd, &pcmd, sizeof(pcmd)) < 0){
			fprintf(stderr, "write pcmd error\n"); 
			break;
		    }
		}
		break;
	   
	    case SDL_KEYDOWN:
		if (event.key.keysym.sym == SDLK_ESCAPE){
		    done = 1;
		}
		break;

	    case SDL_JOYBUTTONDOWN:
		//	if(SDL_JoystickGetButton(joystick, 16)){ 
		//  done = 1;
		//	}
		if(SDL_JoystickGetButton(joystick, 1)){ 
		    printf("stop\n");	    
		    NeckSpeed=0;

		}
		if(SDL_JoystickGetButton(joystick, 4)){ 
		    printf("turn left\n");
		    NeckSpeed = 400;
		}
		if(SDL_JoystickGetButton(joystick, 5)){ 
		    printf("turn right\n");
		    NeckSpeed = -400;
		}	
		break;
	
	    case SDL_JOYAXISMOTION:
		int joyaxis;
	
		if(SDL_JoystickGetAxis(joystick,0)>=+32767) {	  
		    printf("right\n");
		
		
	  
		}

		if(SDL_JoystickGetAxis(joystick,0)<=-32767) {
	  
		    printf("left\n");
		   

		}
		if(SDL_JoystickGetAxis(joystick,1)>=+32767) {
		    printf("down\n");

	    
		}
		if(SDL_JoystickGetAxis(joystick,1)<=-32767) {
		    printf("up\n");
		}


		break;
	    }

	}
	///////////////////
	/*	person_position_x=160;
	if(person_position_x<80){
	    NeckSpeed = 1000;
	}

	else if(person_position_x>240){
	    NeckSpeed = -1000;
	}
	else {
	    NeckSpeed = 0;
	}

	cout<<person_position_x<<endl;
	*/
	///////////////////

	if (ioctl(pfd, URBTC_CONTINUOUS_READ) < 0){
	    fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
	    break;
	}

	read(pfd,&pbuf,sizeof(pbuf));
	//入力するエンコーダのチャンネル指定（scoutと違う）
	pan_angle=pbuf.ct[1];

	pan_degree=pan_angle*(-1)*90/4200.0;

	//	printf("%d %lf\n",pan_angle,pan_degree);


	if((pan_angle>=3000)&&(NeckSpeed<0)){
	    p_velocity=32768+200;
	}

	else if((pan_angle<=-3000)&&(NeckSpeed>0)){
	    p_velocity=32768-200; 
	}
	else {
	    p_velocity=32768+NeckSpeed;	     
	}

	pcmd.offset[2]=p_velocity;

	if (ioctl(pfd, URBTC_COUNTER_SET) < 0){
	    fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
	    break;
	}

	if (write(pfd, &pcmd, sizeof(pcmd)) < 0){
	    fprintf(stderr, "write pcmd error\n"); 
	    break;
	}


	//////////////////
    }




}
int gluiThread1(void* args){
    pantilt();
}


void pantilt_start(){
    th1 = SDL_CreateThread(gluiThread1, NULL);
}
/*
int main(){
	pantilt_init();
	pantilt();
	pantilt_end();
	return 0;

}*/




int pantilt(int vel){
    //ここから初期化


    //ここからループ
    //    qDebug("PanTilt-connect");
    if (ioctl(pfd, URBTC_CONTINUOUS_READ) < 0){
	fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
	//break;
    }

    read(pfd,&pbuf,sizeof(pbuf));
    //入力するエンコーダのチャンネル指定（scoutと違う）
    pan_angle=pbuf.ct[1];

    //	printf("%d\n",pan_angle);
    person=person_position_x;

    p_velocity=32768+vel;	     

    pcmd.offset[2]=p_velocity;

    if (ioctl(pfd, URBTC_COUNTER_SET) < 0){
	fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
	//	break;
    }

    if (write(pfd, &pcmd, sizeof(pcmd)) < 0){
	fprintf(stderr, "write pcmd error\n"); 
	//	break;
    }
    printf("\n");
 
}
