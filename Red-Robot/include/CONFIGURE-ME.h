#include <stdint.h>

/* Edit this integer to be equal to your team number.
*/
const uint8_t TEAM_NUMBER = 25;

/* Edit this string to include your team name! When you connedct your robot to a Bluetooth,
 * controller app, the ID that will show up is "RR-[TEAM NAME]-MAC."

   MAX 15 CHARACTERS
*/
const char *TEAM_NAME = "teddy";

/* Comment this out to receive less verbose debug packets over serial.*/
// Off: it prints a line on EVERY motor write, which floods the line
// follower's [LF] / [CAL] / [TEST] output and slows the control loop.
// #define REDROBOT_DEBUG 1