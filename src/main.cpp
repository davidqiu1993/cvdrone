#include "ardrone/ardrone.h"
#include "functionalities/functionalities.h"

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char **argv)
{
  // Activate the program of locating a colored object
  int exit_code = LocateColoredObject(argc, argv);

  // Program exit
  return exit_code;
}
