#include "MobileCart.h"
#include <math.h>
#include <qpoint.h>
#define ST             32768
#define ST_16BIT       65535
#define DIAMETER       345.44			//���شֵ�Υ
#define WHEEL_DIAMETER 203			//����ľ��
#define WHEEL_ENC      48300			//���ذ��������Υ��󥳤���
#define PI             3.14159265
#define PAN90          29000			//��90�٤Υ��󥳤���
#define Pg             1000                     //��Ū�ϤΥݥƥ󥷥��(���̡�)
#define Po	       2                        //�㳲ʪ�Υݥƥ󥷥�� ��(���̡�)
#define Ps	       20                       //�㳲ʪ�Υݥƥ󥷥�� �ᤤ(���̡�)
#define Pm	       10                       //�㳲ʪ�Υݥƥ󥷥�� �ᤤ(���̡�)
#define Pbg	       5         	        //�㳲ʪ�Υݥƥ󥷥�� �ᤤ(���̡�)
#define CENTER	       225			//�졼���濴�������濴�ޤǤε�Υ
#define RAD2DEG	       (180.0/M_PI)		//RAD����DEG�ؤ��Ѵ� 
#define DANGER         250			//����Υ

void velocity(int lv,int rv);	
void velocity_normal(int lv,int rv);
void velocity_calculate(int lv,int rv);
void velocity_write();
void count();
void stop(); 
void acceleration(int la,int ra);
void distance_xy(int v, double x, double y, int dis);
void degree_deg(int deg);
void connect_robot(char *ldev, char *rdev);
void initial_setting();
void robot_state(int ld_enc,int rd_enc);
void following_course(double *x, double *y, int num);
void initial_position();

int lfd,rfd;						//�����ΥǥХ��������ץ���
long left_rotate,right_rotate;				//�����β�ž����
int l_velocity,r_velocity;				//�����μ��ؤ�®��
int l_ac,r_ac;						//�����μ��ؤβ�®��
long g_time,g_time_old;					//�񤭹�����Ȥ��λ���(���ߤ���)
double state_x,state_y,degree;				//��ܥåȤμ��ʰ��֡�����
long l_wheel,r_wheel,l_wheel_old,r_wheel_old;		//���ؤβ�ž��(���ߤ���)
double sx[2],sy[2],sd[2];				//��ܥåȤμ��ʰ���(���ߤ���)

struct ccmd lcmd;
struct ccmd rcmd;

struct uin lbuf;
struct uin rbuf;
struct uout obuf;
FILE *fp,*fp1;

static char *ldevfiles = {"/dev/urbtc1"};
static char *rdevfiles = {"/dev/urbtc0"};

void MobileCart::run()
{
   double goal_x=0,goal_y=0;                       //��Ū�Ϥκ�ɸ
   double goal_len=0;                              //��Ū�ϤޤǤε�Υ
   double distance;                                //�㳲ʪ�ޤǤκ�û��Υ
   double robot_position_x=0,robot_position_y=0;   //��ܥåȤκ�ɸ
   double direction_x,direction_y;                 //��ܥåȤοʹ�����
   int i;
   QPoint *obpt=new QPoint[681];		   //sensor data
   QPoint *bg_obpt=new QPoint[100];		   //sensor data
   double RP_x[100000],RP_y[100000];		   //�����ϩ�ǡ�����Ǽ
   connect_robot(ldevfiles,rdevfiles);
   initial_setting(); 
   int R=0,L=0;
   int pan_deg=0;
   int max_velocity=0;
   for(i=0;i<100;i++){
      bg_obpt[i].setX(i*4-200);
      bg_obpt[i].setY(-400);
   } 
   qDebug("Mobile cart-connect");
  /*------�ǡ�����Ͽ----------*/  
  if((fp=fopen("simpos.txt","a"))==NULL)
    fprintf(stderr,"error\n");
  if((fp1=fopen("robopos.txt","a"))==NULL)
    fprintf(stderr,"error\n");
  //fprintf(fp1,"x\ty\tdeg\n");
  //fprintf(fp,"x\ty\n");
  /*---------------------------*/


   while(1){
      count();
      pan_deg=pan_angle;
      //max_velocity=(int)person_distance*4;
      //if(max_velocity>1000)
	max_velocity=800;
      //�㳲ʪ�ʤ����о�ȯ���桢�󤬰۾���٤Ǥʤ�
      if(RobotState==true && Detection==true && (pan_deg>-ST || pan_deg<ST)){

	 //�оݤޤǤε�Υ���������٤���
	 if(person_distance>90){
	    /*----------------�ѥ�����1--------------------*/
	    /*if(pan_deg>0 && pan_deg<=PAN90){
	      L = 400 - 400*pan_deg/PAN90;
	      velocity(L,400);
	      }
	      else if(pan_deg<=0 && pan_deg>=-PAN90){
	      R=400 + pan_deg*400/PAN90;
	      velocity(400,R);
	      }
	      else if(pan_deg>=PAN90 && pan_deg<31000){
	      velocity(-100,100);
	      }
	      else if(pan_deg<=-PAN90 && pan_deg>-31000){
	      velocity(100,-100);
	      }
	      else{  
	    //velocity(0,0);
	    stop();
	    }*/

	    /*----------------�ѥ�����2--------------------*/
	    //qDebug("normal");
	    if(pan_deg>0 && pan_deg<29000){
	       L = max_velocity - max_velocity *pan_deg/29000;
	       velocity_normal(L,max_velocity);
	    }
	    else if(pan_deg<=0 && pan_deg>-29000){
	       R=max_velocity + pan_deg*max_velocity/29000;
	       velocity_normal(max_velocity,R);
	    }
	    else if(pan_deg>=29000 && pan_deg<33000){
	       velocity_normal(-400,400);
	    }
	    else if(pan_deg<=-29000 && pan_deg>-33000){
	       velocity_normal(400,-400);
	    }
	    else{  
	       velocity_normal(0,0);
	       stop();
	    }

		if(danger_distance==true){
			stop();
                }
	 }
	 //else if(person_distance<=90 && person_distance>10){
	 //velocity(-300,-300);
	 //}
	 else{
	    //qDebug("stop");
	    velocity(0,0);
	    stop();
	    //lw=rw=0;
	 }
      }
      //����⡼�� �о�ȯ���桢�㳲ʪ����
      else if(/*Detection==true && */RobotState==false){ 
	 qDebug("Mobile Cart:�㳲ʪ����⡼��");
	 velocity(0,0);
	 stop();
   //      initial_setting();
         initial_position();
	 count();
	 simcount=0;
	 obpt=points;
	 robot_position_x=0;          //mm
	 robot_position_y=0;          //mm
	 goal_len=person_distance*10;       //cm*10
	 goal_x=goal_len*sin(PI*pan_deg/58000);   //mm
	 goal_y=goal_len*cos(PI*pan_deg/58000);   //mm
	 //goal_x=100;   //mm
	 //goal_y=1500;   //mm
	 printf("goal_x=%f\tgoal_y=%f\n",goal_x,goal_y);
	 robot_posV[0].setX(-(int)robot_position_x/5+400);
	 robot_posV[0].setY(-(int)robot_position_y/5+400);

	 /*-------------------------------�����ϩ�η׻�-------------------*/
	 /*��ܥåȤ�(0,0)�ΰ��֤ˤ���Ȥ��Ʒ׻����Ƥ���			*/
	 /*y��ɸ����CENTER������Τ��濴���֤�y�����Τߤ���Ƥ��뤫��	*/
	 /*�׻���̤ϥǡ�����Ǽ���˺�ɸ�Ѵ����Ƥ���(��ž������ʿ�԰�ư)	*/
	 /*----------------------------------------------------------------*/
	 for(;;){
	    obpt=points;
	    goal_len = sqrt((goal_x-robot_position_x)*(goal_x-robot_position_x)+(goal_y-robot_position_y)*(goal_y-robot_position_y));

	    if(goal_len<400 || ExitState==false){
	       qDebug("Mobile Cart:��ϩõ����λ");
	       break;
	    }
	    if(simcount>50000){
	       qDebug("Mobile Cart:��ϩ�ʤ�");
	       break;
	    }
	    direction_x = Pg * (goal_x - robot_position_x) / (goal_len * goal_len);  //mm
	    direction_y = Pg * (goal_y - robot_position_y) / (goal_len * goal_len);  //mm

	    for(i=0;i<681;i++){
              if(abs((int)(obpt[i].x()-goal_x))>150 || abs((int)(obpt[i].y()-goal_y))>150){
	       if(obpt[i].isNull() != TRUE){
		  distance=sqrt((robot_position_x-obpt[i].x())*(robot_position_x-obpt[i].x())+(robot_position_y-obpt[i].y()-CENTER)*(robot_position_y-obpt[i].y()-CENTER));
		  
		  if(distance<200){
		     direction_x += Ps * (robot_position_x - obpt[i].x()) / (distance * distance);  //mm
		     direction_y += Ps * (robot_position_y - obpt[i].y()-CENTER) / (distance * distance);  //mm
		  }
                  else if(distance>=200 && distance<400){
                     if(i<100 || i>580){
		        direction_x += Pbg * (robot_position_x - obpt[i].x()) / (distance * distance);  //mm
		        direction_y += Pbg * (robot_position_y - obpt[i].y()-CENTER) / (distance * distance);  //mm
		     }
		     else{
			direction_x += Pm * (robot_position_x - obpt[i].x()) / (distance * distance);  //mm
		        direction_y += Pm * (robot_position_y - obpt[i].y()-CENTER) / (distance * distance);  //mm
		     }
		  }
		  else if(distance>=400 && distance<1000){
		     direction_x += Po * (robot_position_x - obpt[i].x()) / (distance * distance);  //mm
		     direction_y += Po * (robot_position_y - obpt[i].y()-CENTER) / (distance * distance);  //mm
		  }
	       }
              }
	    }
	    for(i=0;i<100;i++){
	       distance=sqrt((robot_position_x-bg_obpt[i].x())*(robot_position_x-bg_obpt[i].x())+(robot_position_y-bg_obpt[i].y()-CENTER)*(robot_position_y-bg_obpt[i].y()-CENTER));
	       direction_x += Pbg * (robot_position_x - bg_obpt[i].x()) / (distance * distance);  //mm
	       direction_y += Pbg * (robot_position_y - bg_obpt[i].y()-CENTER) / (distance * distance);  //mm
	    }
            //fprintf(fp,"nx=%f\tny=%f\n",robot_position_x,robot_position_y);
	    robot_position_x=robot_position_x+direction_x;
	    robot_position_y=robot_position_y+direction_y;
	    //RP_x[simcount]=robot_position_x*cos(degree)+robot_position_y*sin(degree)+state_x;  //��ɸ�Ѵ�
	    //RP_y[simcount]=-robot_position_x*sin(degree)+robot_position_y*cos(degree)+state_y; //��ɸ�Ѵ�
	    RP_x[simcount]=robot_position_x;
	    RP_y[simcount]=robot_position_y;
	    robot_posV[simcount].setX((int)(-RP_x[simcount]/5+400)); //ɽ����
	    robot_posV[simcount].setY((int)(-RP_y[simcount]/5+600)); //ɽ����
	    simcount++;
	    /*if(sqrt((RP_x[simcount]-state_x)*(RP_x[simcount]-state_x)+(RP_y[simcount]-state_y)*(RP_y[simcount]-state_y))>100){
	       distance_xy(200,RP_x[simcount],RP_y[simcount],10);
	       //break;
	    }*/
	 }
     /*--------------------�ǡ�����¸---------------------*/
         /*for(i=0;i<simcount-1;i++){
	   fprintf(fp,"%.2f\t%.2f\n",RP_x[i],RP_y[i]);
         }
         fprintf(fp,"100000\t0\n");*/


      /*--------------------------����ư��------------------------*/

         if(simcount<50000)
	    following_course(RP_x,RP_y,simcount);
         else
	   degree_deg(20);
            
      }
      //�оݿ�ʪLOST
      else if(Detection==false){
	 velocity_normal(0,0);
	 //qDebug("Mobile Cart: Target Lost");
	 stop();
      }
      else{
	 velocity_normal(0,0);
	 stop();
      }

      //�ܥ��󲡤��ƻߤ᤿
      if(MobState==FALSE)  {   
	 qDebug("Mobile Cart:Stop");
	 stop();
	 mobilecart.wait();
      }

      //printf("%d\t%d\n",person_position_x,person_position_y);

      //��λ�ܥ��󲡤���
      if( ExitState == false )  break;  
   }
   velocity(0,0);
   stop();
   close(lfd);
   close(rfd);
   fclose(fp1);
   fclose(fp);
   qDebug("Mobile Robot Disconnect");
}

/*--------------------------����ư��------------------------*/
/*��ư��Υ��100mm�ʲ��Ϲ��ޤ����ʤ�			  */
/*----------------------------------------------------------*/
void following_course(double *x, double *y, int num)
{
   int i;
   qDebug("Mobile Cart:�㳲ʪ���򳫻�");
   for(i=0;i<num-1;i++){
      /*if(danger_distance==true){
	 stop();
	 qDebug("����Υ");
	 break;
      }*/
      if(sqrt((x[i]-state_x)*(x[i]-state_x)+(y[i]-state_y)*(y[i]-state_y))>150)
	 distance_xy(200,x[i],y[i],10);

      if(/*Detection==false ||*/ MobState==FALSE || ExitState==false /*|| RobotState==true*/){
	 l_velocity=ST;
	 r_velocity=ST;
	 break;
      }
      if(RobotState==true)
	 break;
   }
   qDebug("Mobile Cart:�㳲ʪ����λ");
   //stop();
}

/*----------------------------���ʰ��ּ���---------------------------------*/
/*count()�ؿ����ͤù���о���˼��ʰ��֤�׻�����			   */
/*-------------------------------------------------------------------------*/
void robot_state(int ld_enc,int rd_enc)
{
   double ld,rd;
   ld=ld_enc*WHEEL_DIAMETER*PI/WHEEL_ENC;
   rd=rd_enc*WHEEL_DIAMETER*PI/WHEEL_ENC;
   //printf("lv=%f\trv=%f\tt=%d\n",lv,rv,step);
   sd[1]=sd[0]+(rd-ld)/DIAMETER;
   sy[1]=sy[0]+(rd+ld)*cos(sd[0]+(rd-ld)/(2*DIAMETER))/2;
   sx[1]=sx[0]+(rd+ld)*sin(sd[0]+(rd-ld)/(2*DIAMETER))/2;
   sd[0]=sd[1];
   sy[0]=sy[1];
   sx[0]=sx[1];
   state_y=sy[0];
   state_x=sx[0];
   degree=sd[0];       
   robotPosition_x=state_x;
   robotPosition_y=state_y;
   robotDegree=degree;
   fprintf(fp1,"%.2f\t%.2f\t%.2f\n",state_x,state_y,degree);
   //printf("x=%f\ty=%f\td=%f\tl=%08ld\tr=%08ld\r",state_x,state_y,degree/*180/PI*/,left_rotate,right_rotate);
}

/*----------------------------���ʰ��ֽ����---------------------------------*/
/*���ʰ��֥ǡ�������������		   				     */
/*---------------------------------------------------------------------------*/
void initial_position()
{
   state_x=0;
   state_y=0;
   degree=0;
   sx[0]=sx[1]=0;
   sy[0]=sy[1]=0;
   sd[0]=sd[1]=0;
   robotPosition_x=0;
   robotPosition_y=0;
   robotDegree=0;
}
/*----------------------------��ܥåȤ���³-------------------------------*/
/*�������ʤ��ȥ�ܥåȤ���³����ʤ�				   */
/*-------------------------------------------------------------------------*/
void connect_robot(char *ldev, char *rdev)
{
   if ((lfd = open(ldev, O_RDWR)) == -1) 
      fprintf(stderr, "%s: Open error\n", ldev);
   if ((rfd = open(rdev, O_RDWR)) == -1) 
      fprintf(stderr, "%s: Open error\n", rdev);

   if (ioctl(lfd, URBTC_CONTINUOUS_READ) < 0)
      fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
   if (ioctl(rfd, URBTC_CONTINUOUS_READ) < 0)
      fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");

   if (read(lfd, &lbuf, sizeof(lbuf)) != sizeof(lbuf)) 
      fprintf(stderr, "Read size mismatch.\n");
   if (read(rfd, &rbuf, sizeof(rbuf)) != sizeof(rbuf)) 
      fprintf(stderr, "Read size mismatch.\n");
}

/*------------------------------------�����------------------------------*/
/*���٤ƤΥǡ���������(��ܥåȤ����������󥿤���)			  */
/*------------------------------------------------------------------------*/
void initial_setting()
{   
   int i;
   lcmd.retval = 0 /* RETURN_VAL */;
   rcmd.retval = 0 /* RETURN_VAL */;
   lcmd.setoffset  = CH0 | CH1 | CH2 | CH3;
   rcmd.setoffset  = CH0 | CH1 | CH2 | CH3;
   lcmd.setcounter = CH0 | CH1 | CH2 | CH3;
   rcmd.setcounter = CH0 | CH1 | CH2 | CH3;
   lcmd.resetint   = CH0 | CH1 | CH2 | CH3;
   rcmd.resetint   = CH0 | CH1 | CH2 | CH3;
   lcmd.selin = SET_SELECT | SET_CH2_HIN; /* AD in:ch0,ch1    ENC in:ch2,ch3*/
   rcmd.selin = SET_SELECT | SET_CH2_HIN; /* AD in:ch0,ch1    ENC in:ch2,ch3*/
   lcmd.dout = 0;
   rcmd.dout = 0;
   lcmd.selout = SET_SELECT | CH2;
   rcmd.selout = SET_SELECT | CH2;
   lcmd.offset[0] = lcmd.offset[1] = lcmd.offset[2] = lcmd.offset[3] = 0x7fff;
   rcmd.offset[0] = rcmd.offset[1] = rcmd.offset[2] = rcmd.offset[3] = 0x7fff;
   lcmd.counter[0] = lcmd.counter[1] = lcmd.counter[2] = lcmd.counter[3] = 0;
   rcmd.counter[0] = rcmd.counter[1] = rcmd.counter[2] = rcmd.counter[3] = 0;
   lcmd.posneg = SET_POSNEG | CH0| CH1 | CH2 | CH3; /*POS PWM out*/
   rcmd.posneg = SET_POSNEG | CH0| CH1 | CH2 | CH3; /*POS PWM out*/
   lcmd.breaks = SET_BREAKS | CH0 | CH1 | CH2 | CH3; /*No Brake*/
   rcmd.breaks = SET_BREAKS | CH0 | CH1 | CH2 | CH3; /*No Brake*/
   lcmd.magicno=0x00;
   rcmd.magicno=0x00;
   lcmd.wrrom=0;
   rcmd.wrrom=0;

   if (ioctl(lfd, URBTC_COUNTER_SET) < 0)
      fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
   if (ioctl(rfd, URBTC_COUNTER_SET) < 0)
      fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");

   if (write(lfd, &lcmd, sizeof(lcmd)) < 0)
      fprintf(stderr, "lcmd write error\n");
   if (write(rfd, &rcmd, sizeof(rcmd)) < 0)
      fprintf(stderr, "rcmd write error\n");

   for (i=0; i<4; i++) {
      obuf.ch[i].x = 0;
      obuf.ch[i].d = 0;
      obuf.ch[i].kp = 0;
      obuf.ch[i].kpx = 1;
      obuf.ch[i].kd = 0;
      obuf.ch[i].kdx = 1;
      obuf.ch[i].ki = 0;
      obuf.ch[i].kix = 1;
   }
   lcmd.setcounter = 0;
   rcmd.setcounter = 0;
   lcmd.offset[2] = ST;
   rcmd.offset[2] = ST;

   if (ioctl(lfd, URBTC_DESIRE_SET) < 0)
      fprintf(stderr, "ioctl: URBTC_DESIRE_SET error\n");
   if (ioctl(rfd, URBTC_DESIRE_SET) < 0)
      fprintf(stderr, "ioctl: URBTC_DESIRE_SET error\n");

   if (ioctl(lfd, URBTC_BUFREAD) < 0)
      fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
   if (ioctl(rfd, URBTC_BUFREAD) < 0)
      fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");

   if (ioctl(lfd, URBTC_CONTINUOUS_READ) < 0)
      fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
   if (ioctl(rfd, URBTC_CONTINUOUS_READ) < 0)
      fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");

   l_velocity=ST;
   r_velocity=ST;
   left_rotate=0;
   right_rotate=0;
   read(lfd,&lbuf,sizeof(lbuf));
   read(rfd,&rbuf,sizeof(rbuf));
   g_time_old=lbuf.time;
   g_time=0;
   state_x=0;
   state_y=0;
   degree=0;
   l_wheel=0;
   r_wheel=0;
   l_wheel_old=0;
   r_wheel_old=0;
   sx[0]=sx[1]=0;
   sy[0]=sy[1]=0;
   sd[0]=sd[1]=0;
   robotPosition_x=0;
   robotPosition_y=0;
   robotDegree=0;
   acceleration(50,50);
}

/*----------------------------��®��®��----------------------------------------*/
/*�����̡��˻���         					                */
/*���Τ߻���     					                        */
/*------------------------------------------------------------------------------*/
void acceleration(int la,int ra)
{
   l_ac=la;
   r_ac=ra;
}

/*--------------------------®�ٽ񤭹���----------------------------------------*/
/*���ꤷ��®�٤�񤭹���							*/
/*����򤷤ʤ��ȷ׻�����®�٤�ư���ʤ�						*/
/*------------------------------------------------------------------------------*/
void velocity_write()
{
   lcmd.offset[2] = l_velocity;
   rcmd.offset[2] = r_velocity;

   if (ioctl(lfd, URBTC_COUNTER_SET) < 0)
      fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
   if (ioctl(rfd, URBTC_COUNTER_SET) < 0)
      fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");

   if (write(lfd, &lcmd, sizeof(lcmd)) < 0)
      fprintf(stderr, "write lcmd error\n");
   if (write(rfd, &rcmd, sizeof(rcmd)) < 0)
      fprintf(stderr, "write rcmd error\n");
}    
/*--------------------------------����®��--------------------------------------*/
/*�����̡��˻���                       						*/
/*��ž®�٤�����ɤ���Ǥ�褤         						*/
/*ñ�̤�mm/s�ǻ��ꤹ�뤳��             						*/
/*------------------------------------------------------------------------------*/
void velocity(int lv, int rv)
{
   int l=0,r=0;
   int t;
   long ld0,rd0;
   static int lvv,rvv;
   int lvv_old=0,rvv_old=0;
   int left_ac,right_ac;
   left_ac=l_ac;
   right_ac=r_ac;
   count();
   t=g_time;
   ld0=left_rotate;
   rd0=right_rotate;
   for(;;){
      count();
      if((g_time-t) >= 10){
	 lvv=(int)((left_rotate-ld0)*WHEEL_DIAMETER*PI*1000/(WHEEL_ENC*(g_time-t)));
	 rvv=(int)((right_rotate-rd0)*WHEEL_DIAMETER*PI*1000/(WHEEL_ENC*(g_time-t)));
	 t=g_time;
	 ld0=left_rotate;
	 rd0=right_rotate;
	 if(abs(lvv-lv)>5 && l==0){
	    left_ac=(int)(500-2500/abs(lvv-lv));
	    if(lvv<lv)  l_velocity=l_velocity-left_ac;
	    else  l_velocity=l_velocity+left_ac;
	 }
	 else
	    l=1;

	 if(abs(rvv-rv)>5 && r==0){
	    right_ac=(int)(500-2500/abs(rvv-rv));
	    if(rvv<rv)  r_velocity=r_velocity-right_ac;
	    else  r_velocity=r_velocity+right_ac;
	 }
	 else
	    r=1;

	 if((lvv>=0 && lvv_old<0) || (lvv<=0 && lvv_old>0)) l_velocity=ST; 
	 if((rvv>=0 && rvv_old<0) || (rvv<=0 && rvv_old>0)) r_velocity=ST; 
	 lvv_old=lvv;
	 rvv_old=rvv;
	 //velocity_calculate(l,r);
	 velocity_write();
	 //printf("l=%08d\tr=%08d\tlv=%d\trv=%d\n",l_velocity,r_velocity,lvv,rvv);
      }
      if(l==1 && r==1) break;
      if(Detection==false || MobState==FALSE || ExitState==false){
	 l_velocity=ST;
	 r_velocity=ST;
	 break;
      }
   }
   if(lv==0) l_velocity=ST;
   if(rv==0) r_velocity=ST;
   velocity_write();
   //printf("\n");
}

/*----------------------------®�ٻ���-------------------------------------*/
/*®�٤���ꤹ�롣��δؿ����쥹�ݥ󥹤��ᤤ  			   */
/*-------------------------------------------------------------------------*/
void velocity_normal(int lv, int rv)
{
   int l,r;
   l=(int)(-0.00000002*abs(lv*lv*lv*lv)+0.00005*abs(lv*lv*lv)-0.0449*abs(lv*lv)-3.7383*abs(lv)+32768);
   r=(int)(-0.00000002*abs(rv*rv*rv*rv)+0.00005*abs(rv*rv*rv)-0.0449*abs(rv*rv)-3.7383*abs(rv)+32768);
   if(lv<=0) l=65575-l;
   if(rv<=0) r=65575-r;
   for(;;){
      count();
      velocity_calculate(l,r);
      velocity_write();
      if(l_velocity==l && r_velocity==r) break;
      if(Detection==false || MobState==FALSE || ExitState==false){
	 l_velocity=ST;
	 r_velocity=ST;
	 break;
      }
   }
   velocity_write();
}

/*----------------------------®�ٷ׻�-------------------------------------*/
/*��ɸ®�٤ˤʤ�ޤǲ�®�٤���®�٤����������			   */
/*-------------------------------------------------------------------------*/
void velocity_calculate(int lv,int rv)
{
   if(l_velocity != lv){
      if(l_velocity > lv){
	 l_velocity=l_velocity-l_ac;
	 if(l_velocity < lv) l_velocity=lv;
      }
      else{              
	 l_velocity=l_velocity+l_ac; 
	 if(l_velocity > lv) l_velocity=lv;
      }
   }
   if(r_velocity != rv){
      if(r_velocity > rv){
	 r_velocity=r_velocity-r_ac; 
	 if(r_velocity < rv) r_velocity=rv;
      }
      else{
	 r_velocity=r_velocity+r_ac; 
	 if(r_velocity > rv) r_velocity=rv;
      }
   }
   //fprintf(stderr,"l=%d\tr=%d\n ",l_velocity,r_velocity);
}

/*---------------------------���������󥿤��ͼ���----------------------------------*/
/*���󥳡��������ͤ��ѿ��˳�Ǽ       					           */
/*����Ū�ˤ����Ū�ˤ�ä��ۤ�������       	    				   */
/*---------------------------------------------------------------------------------*/
void count()
{
   int l,r,t;
   if (ioctl(lfd, URBTC_CONTINUOUS_READ) < 0)
      fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
   if (ioctl(rfd, URBTC_CONTINUOUS_READ) < 0)
      fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");

   read(lfd,&lbuf,sizeof(lbuf));
   read(rfd,&rbuf,sizeof(rbuf));

   if(lbuf.time<g_time_old)  t=(ST_16BIT-g_time_old)+lbuf.time+1;
   else                      t=lbuf.time-g_time_old;

   if(lbuf.ct[0]>0 && l_wheel_old<0 && lbuf.ct[0]>30000)       l=-(32767-lbuf.ct[0])-(32768+l_wheel_old)-1;
   else if(lbuf.ct[0]<0 && l_wheel_old>0 && lbuf.ct[0]<-30000) l=(32768+lbuf.ct[0])+(32767-l_wheel_old)+1;
   else                                                        l=lbuf.ct[0]-l_wheel_old;

   if(rbuf.ct[0]>0 && r_wheel_old<0 && rbuf.ct[0]>30000)       r=-(32767-rbuf.ct[0])-(32768+r_wheel_old)-1;
   else if(rbuf.ct[0]<0 && r_wheel_old>0 && rbuf.ct[0]<-30000) r=(32768+rbuf.ct[0])+(32767-r_wheel_old)+1;
   else                                                        r=rbuf.ct[0]-r_wheel_old;


   g_time=g_time+t;
   l_wheel=l_wheel+l;
   r_wheel=r_wheel+r;

   left_rotate=-l_wheel;
   right_rotate=-r_wheel;
   robot_state(-l,-r);

   g_time_old=lbuf.time;
   l_wheel_old=lbuf.ct[0];
   r_wheel_old=rbuf.ct[0];
}

/*-------------------------------------�۵����-----------------------------------*/
/*����ߤ���                   				                          */
/*���̤˻ߤޤꤿ���Ȥ���®�ٻ����0mm/s�򤷤��夬�褤				  */
/*--------------------------------------------------------------------------------*/
void stop()
{
   l_velocity = ST;
   r_velocity = ST;
   velocity_write();
}

/*------------------------------��Υ���ֻ���------------------------------------*/
/*�����ž®�٤������ꤹ��ͽ�� (int v)                               		*/
/*�ޤ�̵���ʤΤ�30000���餤����ꤷ�Ȥ��Ȱ¿�                     		*/
/*������֤�(0,0)�Ȥ�����ɸ��ư����(x,y)�����                    		*/
/*��ɸ�����Ȥθ�(��Υ)�����(���ΤȤ���äƤʤ� int dis)             		*/
/*ñ�̤�mm�ǻ���                                                  		*/
/*���������Υ��0�ˤ���Ȥ��ĤޤǤ⽪��ʤ���                     		*/
/*10mm���餤�Ǥ褤���ޤ���ɸ��1m�ֳ֤�ư�������ʤ�1000�Ȥ���Ф褤		*/
/*------------------------------------------------------------------------------*/
void distance_xy(int v, double x, double y, int dis)
{
   double degree_pos;        //robot now
   double degree_pos_old=0;    //robot old
   double deg;               //goal now
   double deg_old=0;         //goal old
   double d0;                //before convert
   double virtual_distance;  //distance between robot to goal
   double virtual_distance_old=0;
   double mv;                //velocity
   double x0,y0;
   double dv;
   int vmax;
   //int vmaxvv=30000;
   int i=0;
   double Kt=5000;
   double Kd=4000;
   //double Kt=2000;
   //double Kd=3000;
   int lv,rv;
   count();
   vmax=30000;
   l_velocity=r_velocity=vmax;
   //printf("a\n");
   for(;;){
      count();
      degree_pos=degree;
      for(;;){
	 if(degree_pos>=2*PI)
	    degree_pos=degree_pos-2*PI;
	 else if(degree_pos<=-2*PI)
	    degree_pos=degree_pos+2*PI;
	 else
	    break;
      }

      x0=x-state_x;
      y0=y-state_y;
      virtual_distance=sqrt(x0*x0+y0*y0);
      d0=atan(x0/y0);

      deg=d0;
      if(x0>=0 && y0<0){
	 deg=d0+PI;
      }
      else if(x0<0 && y0<0){
	 deg=d0-PI;
      }
      if((degree_pos<=0 && deg>0) || (degree_pos>0 && deg<=0)){
	 if(degree_pos-deg>PI || degree_pos-deg<-PI){
	    if(deg<0)  deg=deg+2*PI;
	    else       deg=deg-2*PI;
	 }
      }
      else{
	 if(degree_pos-deg>PI || degree_pos-deg<-PI){
	    if(deg<0)  deg=deg-2*PI;
	    else       deg=deg+2*PI;
	 }
      }
      d0=deg;

      deg_old=d0;
      if(i==0){
	degree_pos_old=degree_pos;
        i=1;
      }
      //dv=Kt*abs((int)(deg-degree_pos))+Kd*abs((int)(degree_pos-degree_pos_old));  //Kt=50
      dv=abs((int)(Kt*(deg-degree_pos)))+abs((int)(Kd*(degree_pos-degree_pos_old)));  //Kt=50
      //printf("%f\n",dv);
      //printf("deg=%f\tdegree_pos=%f\r",deg,degree_pos);
      if(dv>(ST-vmax)*2)
	 dv=(ST-vmax)*2;
      if(virtual_distance>1000)
	 virtual_distance=1000;

      mv=virtual_distance;
      if(virtual_distance<700)
	 mv=700;

      //printf("dv=%f\tgoal=%f\trobo=%f\tx=%f\ty=%f\n",dv,deg,degree_pos,state_x,state_y);

      //printf("%f\n",mv);
      if(deg<degree_pos){
	 lv=ST-(int)(mv*3+dv);
	 rv=ST-(int)(mv*3-dv);
	 if(rv>ST_16BIT-2000-vmax)
	    rv=ST_16BIT-2000-vmax;
	 if(lv>ST_16BIT-2000-vmax)
	    lv=ST_16BIT-2000-vmax;
	 if(lv<vmax)
	    lv=vmax;
	 if(rv<vmax)
	    rv=vmax;
      }
      else{
	 lv=ST-(int)(mv*3-dv);
	 rv=ST-(int)(mv*3+dv);
	 if(lv>ST_16BIT-2000-vmax)
	    lv=ST_16BIT-2000-vmax;
	 if(rv>ST_16BIT-2000-vmax)
	    rv=ST_16BIT-2000-vmax;
	 if(rv<vmax)
	    rv=vmax;
	 if(lv<vmax)
	    lv=vmax;
      }
      /*if(deg<degree_pos){
	 lv=(int)(vmaxvv-dv);
	 rv=(int)(vmaxvv+dv);
	 if(rv>ST_16BIT-vmax)
	    rv=ST_16BIT-vmax;
	 if(lv>ST_16BIT-vmax)
	    lv=ST_16BIT-vmax;
	 if(lv<vmax)
	    lv=vmax;
	 if(rv<vmax)
	    rv=vmax;
      }
      else{
	 lv=(int)(vmaxvv+dv);
	 rv=(int)(vmaxvv-dv);
	 if(lv>ST_16BIT-vmax)
	    lv=ST_16BIT-vmax;
	 if(rv>ST_16BIT-vmax)
	    rv=ST_16BIT-vmax;
	 if(rv<vmax)
	    rv=vmax;
	 if(lv<vmax)
	    lv=vmax;
      }*/

      //printf("%d %d\n",lv,rv);
      for(;;){
	 velocity_calculate(lv,rv);
	 velocity_write();
	 if(l_velocity==lv && r_velocity==rv) break;
      }
      /*if(danger_distance==true){
	 stop();
	 qDebug("����Υ����");
	 //degree_deg(20);
	 break;
      }*/
      if(RobotState==true)
	 break;

      if((abs((int)(x-state_x))<30 && abs((int)(y-state_y))<30) || virtual_distance<30){
	 break;
      }
      if(/*Detection==false ||*/ MobState==FALSE || ExitState==false /*|| RobotState==true*/){
	 l_velocity=ST;
	 r_velocity=ST;
	 break;
      }
      virtual_distance_old=virtual_distance;
      degree_pos_old=degree_pos;
      //printf("x=%f\ty=%f\tl=%ld\tr=%ld\r",state_x,state_y,l_wheel,r_wheel);
      //velocity_write();
   }
   //printf("\n");
   /*l_velocity=ST;
     r_velocity=ST;*/
     velocity_write();
}
/*---------------------------���ٻ���-------------------------------------------*/
/*����ǲ�ž���٤����  	   						*/
/*����������          	     							*/
/*�颪������               							*/
/*------------------------------------------------------------------------------*/
void degree_deg(int deg)
{
   double d1,deg_pos;
   int v=200,lv,rv;
   //deg=(ld-rd)/DIAMETER;
   v=(int)(-0.00000002*abs(v*v*v*v)+0.00005*abs(v*v*v)-0.0449*abs(v*v)-3.7383*abs(v)+32767);
   if(deg<0){
      rv=ST_16BIT-v;
      lv=v;
   }
   else{
      lv=ST_16BIT-v;
      rv=v;
   }
   count();
   d1=degree*180/PI+deg;
   for(;;){
      count();
      deg_pos=degree*180/PI;
      if((abs((int)d1)-abs((int)deg_pos))>=1){
	 if(deg<0){
	    l_velocity=lv+(ST-500-v)/(abs((int)d1)-abs((int)deg_pos)); 
	    r_velocity=rv-(ST-500-v)/(abs((int)d1)-abs((int)deg_pos)); 
	 }
	 else{
	    l_velocity=lv-(ST-500-v)/(abs((int)d1)-abs((int)deg_pos)); 
	    r_velocity=rv+(ST-500-v)/(abs((int)d1)-abs((int)deg_pos)); 
	 }
      }
      //printf("deg=%f\tdegpos=%f\tlv=%d\trv=%d\n",d1,deg_pos,l_velocity,r_velocity);
      if((abs((int)d1)-abs((int)deg_pos))<3)  break;
      if(Detection==false || MobState==FALSE || ExitState==false){
	 l_velocity=ST;
	 r_velocity=ST;
	 break;
      }

      velocity_write();

   }
   l_velocity=ST;
   r_velocity=ST;
   velocity_write();
}


