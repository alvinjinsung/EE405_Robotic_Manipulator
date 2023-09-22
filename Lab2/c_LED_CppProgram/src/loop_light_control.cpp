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

  gpio_export(30);
  gpio_export(31);

/*
  2. Set direction of GPIO 30 & GPIO31 to out.

  !! REPLACE THIS PART TO YOUR CODE !!

*/

  gpio_set_dir(30, 1);
  gpio_set_dir(31, 1);

/*
  3. Open GPIO30 & GPIO31.

  !! REPLACE THIS PART TO YOUR CODE !!

*/

  fd_gpio_30 = gpio_fd_open(30);
  fd_gpio_31 = gpio_fd_open(31);

// 4. Save the start time(before the light control loop)
  gettimeofday(&s_time,&tz);

/*
  5. LED Control loop for 10 times

  !! REPLACE THIS PART TO YOUR CODE !!

*/

  for (int i=0; i<10; i++) {
    gpio_fd_set_value(fd_gpio_30, 1);
    gpio_fd_set_value(fd_gpio_31, 1);

    gpio_fd_set_value(fd_gpio_30, 0);
    gpio_fd_set_value(fd_gpio_31, 0);
  }

// 6. Save the end time(after the light control loop)

  gettimeofday(&e_time,&tz);
  
  start_time = s_time.tv_sec + s_time.tv_usec*1e-6;
  end_time = e_time.tv_sec + e_time.tv_usec*1e-6;
  printf("start: %.6f end: %.6f\n", start_time, end_time);

/*
  7. Close GPIO30 & GPIO31

  !! REPLACE THIS PART TO YOUR CODE !!

*/

  gpio_fd_close(fd_gpio_30);
  gpio_fd_close(fd_gpio_31);

/*
  8. Set direction of GPIO30 & GPIO31 to in.

  !! REPLACE THIS PART TO YOUR CODE !!

*/

  gpio_set_dir(30, 0);
  gpio_set_dir(31, 0);

/*
  9. Unexport GPIO30 & GPIO31

  !! REPLACE THIS PART TO YOUR CODE !!

*/

  gpio_unexport(30);
  gpio_unexport(31);

  return 0;
}
