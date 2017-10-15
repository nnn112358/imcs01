#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>         /* for read, write, close */
#include <sys/ioctl.h>      /* for ioctl */
#include <sys/types.h>      /* for open */
#include <sys/stat.h>       /* for open */
#include <fcntl.h>          /* for open */

#include <signal.h>

#include "driver/urbtc.h"          /* Linux specific part */
#include "driver/urobotc.h"        /* OS independent part */

void exit_program();

int quit_flag = 1;
int fd;


void exit_program(int sig){
  quit_flag = 0;
  fprintf(stderr, "kill signal is received\n");
}



int main(int argc, char **argv){

  struct uin ibuf;
  struct uout obuf;
  struct ccmd cmd;
  int i;

  char *dev = "/dev/urbtc1";
  
  signal(SIGINT, exit_program);


  if ((fd = open(dev, O_RDWR)) == -1) {
    fprintf(stderr, "%s: Open error\n", dev);
    exit(1);
  }

  if (ioctl(fd, URBTC_CONTINUOUS_READ) < 0){
    fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
    exit(1);
  }
  
    if (ioctl(fd, URBTC_BUFREAD) < 0){
    fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
    exit(1);
  }
  
  
  if (read(fd, &ibuf, sizeof(ibuf)) != sizeof(ibuf)) 
	fprintf(stderr, "Read size mismatch.\n");
	
  cmd.retval = 0 ;
  cmd.setoffset  = CH0 | CH1 | CH2 | CH3;
  cmd.setcounter = CH0 | CH1 | CH2 | CH3;
  cmd.resetint   = CH0 | CH1 | CH2 | CH3;
  cmd.selin = SET_SELECT |  CH0 | CH1 | CH2 | CH3; 
  cmd.dout = 0;
  cmd.selout = SET_SELECT|CH0 | CH1 ; 
  cmd.offset[0] = cmd.offset[1] = cmd.offset[2] = cmd.offset[3] = 0x7fff;
  cmd.counter[0] = cmd.counter[1] = cmd.counter[2] = cmd.counter[3] = 0;
  cmd.magicno = 0x00;
  cmd.posneg = SET_POSNEG | CH0 | CH1 | CH2 | CH3;
  cmd.breaks = SET_BREAKS | CH0 | CH1 | CH2 | CH3;
  cmd.magicno=0x00;
  cmd.wrrom = 0;

  if (ioctl(fd, URBTC_COUNTER_SET) < 0){
    fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
    exit(1);
  }
  if (write(fd, &cmd, sizeof(cmd)) < 0) {
    fprintf(stderr, "write error\n");
    exit(1);
  }

  cmd.setcounter = 0;

  if (write(fd, &cmd, sizeof(cmd)) < 0) {
    fprintf(stderr, "write error\n");
    exit(1);
  }

  if (ioctl(fd, URBTC_DESIRE_SET) < 0)
      fprintf(stderr, "ioctl: URBTC_DESIRE_SET error\n");
      
  if (ioctl(fd, URBTC_BUFREAD) < 0)
	fprintf(stderr, "ioctl: URBTC_BUFREAD error\n");  
      
  if (ioctl(fd, URBTC_CONTINUOUS_READ) < 0){
    fprintf(stderr, "ioctl: URBTC_CONTINUOUS_READ error\n");
    exit(1);
  }




  i = 0;
  while(quit_flag) {
     unsigned int mot_l=32768;
     unsigned int mot_r=32768;

	int speed=15000;

	cmd.offset[0] = mot_l-speed;
	cmd.offset[1] = mot_l+speed;

	if (ioctl(fd, URBTC_COUNTER_SET) < 0){
	fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
	exit(1);
	}

	if (write(fd, &cmd, sizeof(cmd)) > 0) {
	i += 1;
	} else {
	fprintf(stderr, "write cmd error\n");
	exit(1);
	}

  struct uin buf;
    if ((i = read(fd, &buf, sizeof(buf))) != sizeof(buf)) {
      fprintf(stderr, "Warning: read size mismatch (%d!=%d).\n", i, sizeof(buf));
    }
 
    printf("left:%5d right:%5d __%d\n",
	   buf.ct[2], buf.ct[1],buf.time);

  }

	cmd.offset[0] = cmd.offset[1] = cmd.offset[2] = cmd.offset[3] = 32768;
	if (ioctl(fd, URBTC_COUNTER_SET) < 0){
	fprintf(stderr, "ioctl: URBTC_COUNTER_SET error\n");
	exit(1);
	}
	if (write(fd, &cmd, sizeof(cmd)) > 0) {
	} 






  close(fd);

  return 0;
}






