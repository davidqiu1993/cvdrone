#include "functionalities.h"

void _ProcessImage(IplImage** ptrImage);
bool _FindColoredObject(IplImage* image, int color_r, int color_g, int color_b, int error, int* delta_px, int* delta_py);
void _MedianFilter(IplImage** ptrImage);
void _MedianFilter_OpenCV(IplImage** ptrImage);
void _GradientDiffuseAndEqualize(IplImage** ptrImage, int threshold);
void _CheckNeighbourPixels(IplImage* image, int* pixelGroup, int currentGroup, int currentPixel, queue<int>& toCheck, int threshold);


/**
 * @param argc The number of arguments from user input.
 * @param argv The argument vector storing the user input arguments.
 * @return Exit code of the program.
 *
 * @brief
 *    The target of this program is to find and locate a colored
 *    object, and then tell its position to the ROS core.
 */
int LocateColoredObject(int argc, char **argv)
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
    _ProcessImage(&image);

    // Find the colored object
    _FindColoredObject(image, 255, 0, 0, 10, &delta_px, &delta_py);

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
* @param ptrImage Pointer to the image being processed.
*
* @brief
*    Process the image obtain from the AR.Drone. And the processed
*    effect on the input image will directly affect the original
*    image.
*/
void _ProcessImage(IplImage** ptrImage)
{
  // Remove the noise from the image
  for (int i = 0; i < 3; ++i) _MedianFilter_OpenCV(ptrImage);
  //_MedianFilter(ptrImage);

  // Gradient diffuse and equalize
  _GradientDiffuseAndEqualize(ptrImage, 10);
}

/**
* @param image The image obtain from the bottom camera.
* @param color_r The depth of red element in expected color. It ranges
*                from 0 to 255.
* @param color_g The depth of green element in expected color. It ranges
*                from 0 to 255.
* @param color_b The depth of blue element in expected color. It ranges
*                from 0 to 255.
* @param error The average tolerable error of the expected color.
* @param delta_px The error in pixel on x-axis of the screen. It is an
*                 output parameter.
* @param delta_py The error in pixel on y-axis of the screen. It is an
*                 output parameter.
* @return An bool indicating if any colored object found on the
*         input image.
*
* @brief
*    Find the colored object from the image obtain from the bottom
*    camera of the AR.Drone.
*/
bool _FindColoredObject(IplImage* image, int color_r, int color_g, int color_b, int error, int* delta_px, int* delta_py)
{
  return false;
}

/**
* @param ptrImage Pointer to the image being performed median
*                 filter algorithm on.
*
* @brief
*    Perform median filter algorithm on a image to remove most of
*    its noise.
*/
void _MedianFilter(IplImage** ptrImage)
{
  IplImage* image = *ptrImage;
  uchar* arr[9];
  for (int i = 0; i < 9; ++i) arr[i] = new uchar[image->nChannels];
  IplImage* image_dst = cvCloneImage(image);

  // Check the image size and channels
  if (image->width < 3 || image->height < 3) return;

  // Filter the image
  //for (int currentBlock = 0; currentBlock <= image->width*image->height; ++currentBlock)
  for (int currentBlock = image->width + 1; currentBlock <= image->width*(image->height - 1) - 1; ++currentBlock)
  {
    // Put the block to the processing array
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < image->nChannels; ++j) {
        arr[i][j] = (image->imageData)[(currentBlock - image->width - 1 + i) * 3 + j];
      }
    }
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < image->nChannels; ++j) {
        arr[i + 3][j] = (image->imageData)[(currentBlock - 1 + i) * 3 + j];
      }
    }
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < image->nChannels; ++j) {
        arr[i + 6][j] = (image->imageData)[(currentBlock + image->width - 1 + i) * 3 + j];
      }
    }

    // Sort the array with selection sort
    int min, idx;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 9; ++j)
      {
        idx = j;
        min = arr[j][i];
        for (int k = j; k < 9; ++k)
        {
          if (arr[k][i] < min)
          {
            min = arr[k][i];
            idx = k;
          }
        }
        arr[idx][i] = arr[j][i];
        arr[j][i] = min;
      }
    }

    // Median the block
    for (int i = 0; i < 3; ++i)(image_dst->imageData)[currentBlock * 3 + i] = arr[5][i];
  }

  // Release the resources
  for (int i = 0; i < 9; ++i) delete arr[i];
  cvReleaseImage(ptrImage);

  // Reset the pointer
  (*ptrImage) = image_dst;
}

/**
* @param ptrImage Pointer to the image being performed median
*                 filter algorithm on.
*
* @brief
*    Perform median filter algorithm on a image to remove most of
*    its noise. This function uses original OpenCV median filter.
*/
void _MedianFilter_OpenCV(IplImage** ptrImage)
{
  IplImage* image = *ptrImage;
  IplImage* image_dst = cvCreateImage(cvSize(image->width, image->height), image->depth, image->nChannels);

  // Use OpenCV median filter
  cvSmooth(image, image_dst, CV_MEDIAN, 9);

  // Release recources
  cvReleaseImage(ptrImage);

  // Reset the pointer
  (*ptrImage) = image_dst;
}

/**
* @param ptrImage Pointer to the image being performed
*                 gradient diffuse selection and pixel
*                 equalization.
* @param threshold Gradient threshold of the diffuse
*                  process. Threshold plays the role as
*                  an average pixel value, and it should
*                  be non-negative number and its recommanded
*                  upper boundary is 255.
*
* @brief
*    Perform gradient diffuse selection algorithm on the
*    input image and equalize the pixel values in each
*    block.
*/
void _GradientDiffuseAndEqualize(IplImage** ptrImage, int threshold)
{
  IplImage* image = *ptrImage;
  int pixelGroupSize = image->width * image->height;
  int* pixelGroup = new int[pixelGroupSize];
  int currentGroup;
  queue<int> toCheck;
  queue<int> currentGroupPixels;
  int* sumChannelValue = new int[image->nChannels];
  int* aveChannelValue = new int[image->nChannels];


  // Reset the pixel group
  for (int i = 0; i < pixelGroupSize; ++i) pixelGroup[i] = -1;

  // Check if the threshold is valid
  if (threshold < 0) return;

  // Loop check each pixel
  currentGroup = 0;
  for (int i = 0; i < pixelGroupSize; ++i)
  {
    // Unprocessed pixel
    if (pixelGroup[i] < 0)
    {
      // Reset to-check queue and pixel queue
      while (!toCheck.empty()) toCheck.pop();
      while (!currentGroupPixels.empty()) currentGroupPixels.pop();

      // Reset pixel value sum
      for (int channel = 0; channel < image->nChannels; ++channel) sumChannelValue[channel] = 0;

      // Push this pixel to the queue
      toCheck.push(i);

      // Assign group number to target pixel
      pixelGroup[i] = currentGroup;

      // Loop find the valid gradient diffuse pixels
      while (!toCheck.empty())
      {
        int currentPixel = toCheck.front();

        // Push the pixel to current group pixels queue
        currentGroupPixels.push(currentPixel);

        // Sum up the pixel values
        for (int channel = 0; channel < image->nChannels; ++channel)
        {
          sumChannelValue[channel] += (uchar)(image->imageData[currentPixel * 3 + channel]);
        }

        // Check the neighbour pixels of to-check queue front
        _CheckNeighbourPixels(image, pixelGroup, currentGroup, currentPixel, toCheck, threshold);

        // Pop the processed pixel from the to-check queue
        toCheck.pop();
      }

      // Equalize pixel values of the current group
      for (int channel = 0; channel < image->nChannels; ++channel)
      {
        aveChannelValue[channel] = sumChannelValue[channel] / currentGroupPixels.size();
      }
      while (!currentGroupPixels.empty())
      {
        // Set the pixel value
        for (int channel = 0; channel < image->nChannels; ++channel)
        {
          image->imageData[currentGroupPixels.front() * 3 + channel] = aveChannelValue[channel];
        }

        // Pop the processed pixel
        currentGroupPixels.pop();
      }

      // Move to the next pixel group
      ++currentGroup;
    }
  }

  // Release resources
  delete pixelGroup;
  delete sumChannelValue;
  delete aveChannelValue;
}

/**
* @param image The target image.
* @param pixelGroup The array indicating the group number of each pixel.
* @param currentGroup The current pixel group number.
* @param currentPixel The pixel currently pointed to.
* @param toCheck The queue of pixels to be check.
* @param threshold The threshold of valid gradient diffuse.
*
* @brief
*    Check if neighbour pixels of the current pixel is within the valid
*    gradient diffuse threshold. And push the valid neighbour pixels
*    into the to-check queue. It is a sub-function called by the gradient
*    diffuse and equalization function.
*/
void _CheckNeighbourPixels(IplImage* image, int* pixelGroup, int currentGroup, int currentPixel, queue<int>& toCheck, int threshold)
{
  bool validGradient;


  // Check the upward boundary
  if (currentPixel >= image->width)
  {
    // Calculate the pixel index
    int targetPixel = currentPixel - image->width;

    // Check the occupation
    validGradient = (pixelGroup[targetPixel] < 0);

    // Check the upward pixel
    if (validGradient)
    {
      for (int channel = 0; channel < image->nChannels; ++channel)
      {
        if (abs(image->imageData[currentPixel * 3 + channel] - image->imageData[targetPixel * 3 + channel]) > threshold)
        {
          validGradient = false;
          break;
        }
      }
    }

    // Check if valid
    if (validGradient)
    {
      // Push to the to-check queue
      toCheck.push(targetPixel);

      // Assign group number to target pixel
      pixelGroup[targetPixel] = currentGroup;
    }
  }

  // Check the downward boundary
  if (currentPixel < image->width*image->height - image->width)
  {
    // Calculate the pixel index
    int targetPixel = currentPixel + image->width;

    // Check the occupation
    validGradient = (pixelGroup[targetPixel] < 0);

    // Check the downward pixel
    if (validGradient)
    {
      for (int channel = 0; channel < image->nChannels; ++channel)
      {
        if (abs(image->imageData[currentPixel * 3 + channel] - image->imageData[targetPixel * 3 + channel]) > threshold)
        {
          validGradient = false;
          break;
        }
      }
    }

    // Check if valid
    if (validGradient)
    {
      // Push to the to-check queue
      toCheck.push(targetPixel);

      // Assign group number to target pixel
      pixelGroup[targetPixel] = currentGroup;
    }
  }

  // Check the left boundary
  if (currentPixel % image->width > 0)
  {
    // Calculate the pixel index
    int targetPixel = currentPixel - 1;

    // Check the occupation
    validGradient = (pixelGroup[targetPixel] < 0);

    // Check the left pixel
    if (validGradient)
    {
      for (int channel = 0; channel < image->nChannels; ++channel)
      {
        if (abs(image->imageData[currentPixel * 3 + channel] - image->imageData[targetPixel * 3 + channel]) > threshold)
        {
          validGradient = false;
          break;
        }
      }
    }

    // Check if valid
    if (validGradient)
    {
      // Push to the to-check queue
      toCheck.push(targetPixel);

      // Assign group number to target pixel
      pixelGroup[targetPixel] = currentGroup;
    }
  }

  // Check the right boundary
  if (currentPixel % image->width < image->width - 1)
  {
    // Calculate the pixel index
    int targetPixel = currentPixel + 1;

    // Check the occupation
    validGradient = (pixelGroup[targetPixel] < 0);

    // Check the right pixel
    if (validGradient)
    {
      for (int channel = 0; channel < image->nChannels; ++channel)
      {
        if (abs(image->imageData[currentPixel * 3 + channel] - image->imageData[targetPixel * 3 + channel]) > threshold)
        {
          validGradient = false;
          break;
        }
      }
    }

    // Check if valid
    if (validGradient)
    {
      // Push to the to-check queue
      toCheck.push(targetPixel);

      // Assign group number to target pixel
      pixelGroup[targetPixel] = currentGroup;
    }
  }
}
