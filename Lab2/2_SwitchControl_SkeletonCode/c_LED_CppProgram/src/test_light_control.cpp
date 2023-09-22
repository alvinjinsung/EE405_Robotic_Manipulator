#include <stdio.h>
#include <string.h>
#include "../include/gpio_control.hpp"

int main(int argc, char *argv[])
{
  int light_id;
  int fd_gpio_30, fd_gpio_31;
  int on_off_int;
  char on_off_str[8];

  /* 0.Print title */
  printf("test_light_control\n");

  /* 1. Set variables: light_id = 1; */
  light_id = 1;

/*
  2. Export GPIO 30 & GPIO31

  !! REPLACE THIS PART TO YOUR CODE !!

*/

/*
  3. Set direction of GPIO 30 & GPIO31 to out.

  !! REPLACE THIS PART TO YOUR CODE !!

*/

/*
  4. Open GPIO30 & GPIO31.

  !! REPLACE THIS PART TO YOUR CODE !!

*/

  /* 5. Loop while light_id > 0 */
  while(light_id > 0)
  {
    /* A. Prompt output */
    printf("Enter light_id and on_off_str: ");
    /* B. Get user input of light_id and on_off_str */
    scanf("%d %7s", &light_id, on_off_str);
    /* C. Check light_id. Break if <= 0. */
    if(light_id <= 0)
      break;
    /* D. Check on_off_str and set on_off_int */
    if(!strcmp(on_off_str, "off"))
      on_off_int = 0;
    else if(!strcmp(on_off_str, "on"))
      on_off_int = 1;
    else
      on_off_int = -1;

    /* 
    
      5-E. Control the LED according to the user input

      !! REPLACE THIS PART TO YOUR CODE !!

    */

  }

/*
  6. Close GPIO30 & GPIO31

  !! REPLACE THIS PART TO YOUR CODE !!

*/

/*
  7. Set direction of GPIO30 & GPIO31 to in.

  !! REPLACE THIS PART TO YOUR CODE !!

*/

/*
  8. Unexport GPIO30 & GPIO31

  !! REPLACE THIS PART TO YOUR CODE !!

*/

  return 0;
}
