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
#include <iostream>
#include <getche.h>
#define SERVERPORT "4950"   // the port users will be connecting to
// initialize variables
int main(int argc, char *argv[])
{

  int sockfd;
	struct addrinfo hints, *servinfo, *p;
	int rv;
	int numbytes;

	if (argc != 2) {
		fprintf(stderr,"usage: talker hostname message\n");
		exit(1);
	}

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET; // set to AF_INET to use IPv4
	hints.ai_socktype = SOCK_DGRAM;

	if ((rv = getaddrinfo(argv[1], SERVERPORT, &hints, &servinfo)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
		return 1;
	}

	// loop through all the results and make a socket
	for(p = servinfo; p != NULL; p = p->ai_next) {
		if ((sockfd = socket(p->ai_family, p->ai_socktype,
				p->ai_protocol)) == -1) {
			perror("talker: socket");
			continue;
		}

		break;
	}

  if (p == NULL) {

		fprintf(stderr, "talker: failed to create socket\n");
		return 2;
	}

  for(p = servinfo; p != NULL; p = p->ai_next) {
		  if ((sockfd = socket(p->ai_family, p->ai_socktype,
				  p->ai_protocol)) == -1) {
			  perror("talker: socket");
			  continue;
		  }

		  break;
	  }

	if (p == NULL) {
		fprintf(stderr, "talker: failed to create socket\n");
		return 2;
	}
// initialize variables and error print
// Get argument of destination IP (argv) of Bone
// Init datagram socket. You will use UDP network.
  while(1)
// Loop start
  {
    // loop through all the results and make a socket
	  

    char a = getch();

	if ((numbytes = sendto(sockfd, &a, sizeof(char), 0, p->ai_addr, p->ai_addrlen)) == -1) {
		perror("talker: sendto");
		exit(1);
	}

	


	// Add your own script written in Lab 4
  	// Please refer to attached code ‘talker.c’ for writing your own script
  	// Print information: key and cmd.
	
	usleep(1000);
  }

  freeaddrinfo(servinfo);
  close(sockfd);

 done:
  return 0;
}