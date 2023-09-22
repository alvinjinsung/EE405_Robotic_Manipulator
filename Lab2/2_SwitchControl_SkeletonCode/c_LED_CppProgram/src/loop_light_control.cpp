#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "../include/gpio_control.hpp"

int main(int argc, char *argv[])
{
  int fd_gpio_30, fd_gpio_31;
  
  struct timeval s_time, e_time;
  struct timezone tz;
  double start_time, end_time;

  /* 0.Print title */
  printf("loop_light_control\n");

/*
  1. Export GPIO 30 & GPIO31

  !! REPLACE THIS PART TO YOUR CODE !!

*/

/*
  2. Set direction of GPIO 30 & GPIO31 to out.

  !! REPLACE THIS PART TO YOUR CODE !!

*/

/*
  3. Open GPIO30 & GPIO31.

  !! REPLACE THIS PART TO YOUR CODE !!

*/

// 4. Save the start time(before the light control loop)
  gettimeofday(&s_time,&tz);

/*
  5. LED Control loop for 10 times

  !! REPLACE THIS PART TO YOUR CODE !!

*/

// 6. Save the end time(after the light control loop)

  gettimeofday(&e_time,&tz);
  
  start_time = s_time.tv_sec + s_time.tv_usec*1e-6;
  end_time = e_time.tv_sec + e_time.tv_usec*1e-6;
  printf("start: %.6f end: %.6f\n", start_time, end_time);

/*
  7. Close GPIO30 & GPIO31

  !! REPLACE THIS PART TO YOUR CODE !!

*/

/*
  8. Set direction of GPIO30 & GPIO31 to in.

  !! REPLACE THIS PART TO YOUR CODE !!

*/

/*
  9. Unexport GPIO30 & GPIO31

  !! REPLACE THIS PART TO YOUR CODE !!

*/

  return 0;
}
