#include "functionalities.h"

void _ProcessImage(IplImage* image);
bool _FindColoredObject(IplImage* image, int* delta_px, int* delta_py);

/**
 * @return Exit code of the program.
 *
 * @brief
 *    The target of this program is to find and locate a colored
 *    object, and then tell its position to the ROS core.
 */
int LocateColoredObject()
{
  const double DEFAULT_MOVE_SPEED = 1.0; // Default movement speed

  ARDrone ardrone; // AR.Drone entity
  IplImage *image = NULL; // Image obtain from the AR.Drone

  double vx, vy, vz, vr; // Expected movement speed vector
  int delta_px, delta_py; // Error vector of the colored object on bottom image
  long tick_time0, tick_time1; // Time on sampling the velocity vector
  double vel_x, vel_y, vel_z; // Velocity vector obtained from AR.Drone sensor
  double pos_x, pos_y, pos_z; // Position vector of the AR.Drone from the origin

  // Initialize
  if (!ardrone.open()) {
    printf("Failed to initialize.\n");
    return -1;
  }
  pos_x = 0; pos_y = 0; pos_z = 0;
  tick_time0 = clock();

  // Battery
  printf("Battery = %d%%\n", ardrone.getBatteryPercentage());

  // Switch to the buttom camera
  ardrone.setCamera(1);

  // Instructions
  printf("***************************************\n");
  printf("*       CV Drone sample program       *\n");
  printf("*           - How to Play -           *\n");
  printf("***************************************\n");
  printf("*                                     *\n");
  printf("* - Controls -                        *\n");
  printf("*    'Space' -- Takeoff/Landing       *\n");
  printf("*    'Up'    -- Move forward          *\n");
  printf("*    'Down'  -- Move backward         *\n");
  printf("*    'Left'  -- Turn left             *\n");
  printf("*    'Right' -- Turn right            *\n");
  printf("*    'Q'     -- Move upward           *\n");
  printf("*    'A'     -- Move downward         *\n");
  printf("*                                     *\n");
  printf("* - Others -                          *\n");
  printf("*    'C'     -- Clear Position        *\n");
  printf("*    'F'     -- Find Colored Object   *\n");
  printf("*    'Esc'   -- Exit                  *\n");
  printf("*                                     *\n");
  printf("***************************************\n\n");

  while (1) {
    // Key input
    int key = cvWaitKey(33);
    if (key == 0x1b) break;

    // Update
    if (!ardrone.update()) break;

    // Get an new image
    if (image != NULL) cvReleaseImage(&image);
    image = cvCloneImage(ardrone.getImage());
    
    // Preprocess the image
    _ProcessImage(image);

    // Find the colored object
    _FindColoredObject(image, &delta_px, &delta_py);

    // Accumulate the position vector
    ardrone.getVelocity(&vel_x, &vel_y, &vel_z);
    tick_time1 = clock();
    pos_x += vel_x * ((double)(tick_time1 - tick_time0)) / (double)(CLOCKS_PER_SEC);
    pos_y += vel_y * ((double)(tick_time1 - tick_time0)) / (double)(CLOCKS_PER_SEC);
    pos_z += vel_z * ((double)(tick_time1 - tick_time0)) / (double)(CLOCKS_PER_SEC);
    tick_time0 = tick_time1;

    // Print the position information of the drone
    printf("pos = (%lf, %lf, %lf)\n", pos_x, pos_y, pos_z);

    // Take off / Landing 
    if (key == ' ') {
      if (ardrone.onGround()) ardrone.takeoff();
      else                    ardrone.landing();
    }

    // Move
    vx = 0; vy = 0; vz = 0; vr = 0;
    if (key == 0x260000 || key == 'w') vx = DEFAULT_MOVE_SPEED;
    if (key == 0x280000 || key == 's') vx = -DEFAULT_MOVE_SPEED;
    if (key == 0x250000 || key == 'a') vy = DEFAULT_MOVE_SPEED;
    if (key == 0x270000 || key == 'd') vy = -DEFAULT_MOVE_SPEED;
    if (key == 'q')                    vr = DEFAULT_MOVE_SPEED;
    if (key == 'e')                    vr = -DEFAULT_MOVE_SPEED;
    if (key == '=')                    vz = DEFAULT_MOVE_SPEED;
    if (key == '-')                    vz = -DEFAULT_MOVE_SPEED;
    ardrone.move3D(vx, vy, vz, vr);

    // Functional commands
    if (key == 'c') {
      pos_x = 0;
      pos_y = 0;
      pos_z = 0;
    }

    // Display the image
    cvShowImage("camera", image);
  }

  // See you
  ardrone.close();

  // Exit in normal mode
  return 0;
}

/**
 * @param image The image to process.
 *
 * @brief
 *    Process the image obtain from the AR.Drone. And the processed 
 *    effect on the input image will directly affect the original 
 *    image.
 */
void _ProcessImage(IplImage* image)
{
  return;
}

/**
 * @param image The image obtain from the bottom camera.
 * @param delta_px The error in pixel on x-axis of the screen. It is 
 *                 an output parameter.
 * @param delta_py The error in pixel on y-axis of the screen. It is 
 *                 an output parameter.
 * @return An bool indicating if any colored object found on the 
 *         input image.
 *
 * @brief
 *    Find the colored object from the image obtain from the bottom 
 *    camera of the AR.Drone.
 */
bool _FindColoredObject(IplImage* image, int* delta_px, int* delta_py)
{
  return false;
}
