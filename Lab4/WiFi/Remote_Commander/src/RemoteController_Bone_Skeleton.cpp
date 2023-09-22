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

#define MYPORT "4950"	// the port users will be connecting to

#define MAXBUFLEN 100
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

// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in*)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}


int main(int argc, char *argv[])
{
 /*0. Print Title
1. Set control parameters – gain etc.
2. Init PWM sysfs.(Optional)
3. Init GPIO_LED  (Optional)
4. Open datagram socket and bind*/
  /* Print Key guide */

  int sockfd;
	struct addrinfo hints, *servinfo, *p;
	int rv;
	int numbytes;
	struct sockaddr_storage their_addr;
	char buf[MAXBUFLEN];
	socklen_t addr_len;
	char s[INET6_ADDRSTRLEN];

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET6; // set to AF_INET to use IPv4
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE; // use my IP

	if ((rv = getaddrinfo(NULL, MYPORT, &hints, &servinfo)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
		return 1;
	}

	// loop through all the results and bind to the first we can
	for(p = servinfo; p != NULL; p = p->ai_next) {
		if ((sockfd = socket(p->ai_family, p->ai_socktype,
				p->ai_protocol)) == -1) {
			perror("listener: socket");
			continue;
		}

		if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
			close(sockfd);
			perror("listener: bind");
			continue;
		}

		break;
	}

	if (p == NULL) {
		fprintf(stderr, "listener: failed to bind socket\n");
		return 2;
	}

	freeaddrinfo(servinfo);

	printf("listener: waiting to recvfrom...\n");

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
    addr_len = sizeof their_addr;
	  if ((numbytes = recvfrom(sockfd, buf, MAXBUFLEN-1 , 0,
		  (struct sockaddr *)&their_addr, &addr_len)) == -1) {
		  perror("recvfrom");
		  exit(1);
	  }

	  printf("listener: got packet from %s\n",
		  inet_ntop(their_addr.ss_family,
			  get_in_addr((struct sockaddr *)&their_addr),
			  s, sizeof s));
	  printf("listener: packet is %d bytes long\n", numbytes);
	  buf[numbytes] = '\0';
	  printf("listener: packet contains \"%s\"\n", buf);

	// Please refer to attached code ‘listener.c’ for writing your own script
 	// use strtok() function to parse command to variables
	// use atoi() function to convert a character string to an integer value
	

    usleep(1000);
  }

  close(sockfd);
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