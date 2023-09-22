#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
// pre-define listener function
 // echo macro code
static void echo(char *str, char *file)
{
  int fd = open(file, O_WRONLY);
  if(fd < 0)
  {
	printf("%s:open error.\n", file);
	exit(-1);
  }
  write(fd, str, strlen(str));
  close(fd);
}
int main(int argc, char *argv[])
{
 /*0. Print Title
1. Set control parameters – gain etc.
2. Init PWM sysfs.(Optional)
3. Init GPIO_LED  (Optional)
4. Open datagram socket and bind*/
  /* Print Key guide */

  printf("-----------------------------------\n");
  printf(" R  : move  1cm in the world z-axis\n");
  printf(" F  : move -1cm in the world z-axis\n");
  printf(" W  : move  1cm in the world z-axis\n");
  printf(" S  : move -1cm in the world z-axis\n");
  printf(" D  : move  1cm in the world z-axis\n");
  printf(" A  : move -1cm in the world z-axis\n");
  printf(" E  : end the teleoperation\n");
  printf(" I  : move to the initial position\n");
  printf("-----------------------------------\n");

  while(1)
  {
	// Please refer to attached code ‘listener.c’ for writing your own script
 	// use strtok() function to parse command to variables
	// use atoi() function to convert a character string to an integer value
	

    usleep(1000);
  }
  /* Stop PWM */
  /* Close socket*/
  /* Close GPIO_LED*/
  /* Close PWM sysfs files*/
  return 0;
}

/* 
Please note that objective of the Lab 4 is communication and camera. So you can ignore optional parts now.
But if you fully implement the file, it will be used in the Lab 5. (But it will not be graded in the Lab 4)
The only thing you should complete on the Lab 4 is \
1. Remote conmtroller sends a keyboard input(maybe single alphabet) in PC
2. Bone receives the keyboard input
3. And finally, shows the keyboard input on the shell of Bone.

You may use printf function to show that your bone received the packet.
In short, you should re-implement talker.c and listener.c with keyboard input. 
(please refer getch function in the Lab 1.)
*/