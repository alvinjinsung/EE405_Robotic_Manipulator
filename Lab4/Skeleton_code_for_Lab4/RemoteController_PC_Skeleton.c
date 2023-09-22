#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/time.h>
#define SERVERPORT "4950"   // the port users will be connecting to
// initialize variables
int main(int argc, char *argv[])
{
// initialize variables and error print
// Get argument of destination IP (argv) of Bone
// Init datagram socket. You will use UDP network.
  while(1)
// Loop start
  {
	// Add your own script written in Lab 4
  	// Please refer to attached code ‘talker.c’ for writing your own script
  	// Print information: key and cmd.
	
	usleep(1000);
  }
 done:
  return 0;
}