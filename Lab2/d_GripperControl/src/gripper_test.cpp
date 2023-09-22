#include <stdio.h>
#include <string.h>
#include <chrono>
#include "../include/gpio_control.hpp"
#include "../include/gripper_control.hpp"

int main(int argc, char *argv[])
{

  /*
    1. Define variables for GPIO port number, GPIO file descriptor and length of time duration for holding an object
    !! FILL WITH YOUR CODE !!
  */

  int gpio = 30;
  int fd_gpio;

  printf("gripper_control\n");

  /*
    2. Open the gripper GPIO port
    !! FILL WITH YOUR CODE !!
  */

  fd_gpio = gripper_open(gpio);

  /* Save start time of the loop */
  std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
  std::chrono::system_clock::time_point current_time;
  std::chrono::microseconds loop_elasped_time_microsec;

  loop_elasped_time_microsec = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time);
  int pickup_time = 10;

  /* Turn on the gripper while pickup_time */
  while(loop_elasped_time_microsec.count()/1e6 < pickup_time)
  {
    current_time = std::chrono::system_clock::now();
    loop_elasped_time_microsec = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time);
    /*
      3. Turn on the gripper
      !! FILL WITH YOUR CODE !!
    */

    gripper_on(fd_gpio);
  }

  /*
    4. Close the gripper GPIO port
    !! FILL WITH YOUR CODE !!
  */

  gripper_off(fd_gpio);
  gripper_close(gpio, fd_gpio);

  return 0;
}