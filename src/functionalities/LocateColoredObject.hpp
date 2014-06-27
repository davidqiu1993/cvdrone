/**
* D.D. Selector stands for "David's Diffuse Selector", or the full
* name as "David's Gradient Diffuse Color Block Selector". It is a
* method managing to select human-recognized color block in a image
* or a video frame, with affection of shadow in limited gradient
* threshold.
*
* This color block detection method can detect a single color block
* applying the D.D. Selector.
*
* The author, David Qiu, designs this method and implements it in
* OpenCV. The author remains all rights of the methods, algorithms,
* and code. One can only access to the code for study purpose.
* Commercial use or releasement related to the methods, algorithms,
* or code is NOT allowed.
*
* Creative Commons: BY-NC-ND
*
* Author:  David Qiu (david@davidqiu.com)
* Website: http://www.davidqiu.com/
*/

#ifndef _LOCATE_COLORED_OBJECT_HPP_
#define _LOCATE_COLORED_OBJECT_HPP_

#include "../ardrone/ardrone.h"
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <queue>
using namespace std;

void _ProcessImage(IplImage** ptrImage);
bool _FindColoredObject(IplImage* image, int* channelValues, int colorThreshold, int gradientThreshold, int* delta_px, int* delta_py);
void _MedianFilter(IplImage** ptrImage);
void _MedianFilter_OpenCV(IplImage** ptrImage);
void _CheckNeighbourPixels(IplImage* image, int* pixelGroup, int currentGroup, int currentPixel, queue<int>& toCheck, int threshold);
void _DrawCrosshair(IplImage* image, int x, int y, int size, int* color);


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
  const double DEFAULT_MOVE_SPEED = 0.6; // Default movement speed

  ARDrone ardrone; // AR.Drone entity
  IplImage *image = NULL; // Image obtain from the AR.Drone
  IplImage *image_process = NULL; // Image to process

  float scale = 0.5; // Scale of processed image
  int expectedColor[3] = { 64, 20, 255 }; // Expected color of the block
  int gradientThreshold = 10;
  int colorThreshold = 30;
  int crosshairColor[3] = { 255, 0, 0 }; // Color of the crosshair on the target
  int crosshairSize = 11; // Size of the crosshair on the target

  bool autoFindObjectFlag; // The flag indicating if auto find object and locate
  double vx, vy, vz, vr; // Expected movement speed vector
  int delta_px, delta_py; // Error vector of the colored object on bottom image
  long tick_time0, tick_time1; // Time on sampling the velocity vector
  double vel_x, vel_y, vel_z; // Velocity vector obtained from AR.Drone sensor
  double pos_x, pos_y, pos_z; // Position vector of the AR.Drone from the origin

  bool blockFoundFlag;

  // Initialize
  if (!ardrone.open()) {
    printf("Failed to initialize.\n");
    return -1;
  }
  pos_x = 0; pos_y = 0; pos_z = 0;
  tick_time0 = clock();
  autoFindObjectFlag = false;

  // Battery
  printf("Battery = %d%%\n", ardrone.getBatteryPercentage());

  // Switch to the buttom camera
  ardrone.setCamera(1);
  
  // Instructions
  printf("*******************************************\n");
  printf("*         CV Drone sample program         *\n");
  printf("*             - How to Play -             *\n");
  printf("*******************************************\n");
  printf("*                                         *\n");
  printf("* - Controls -                            *\n");
  printf("*    'Space'     -- Takeoff/Landing       *\n");
  printf("*    '¡ü' / 'W'  -- Move forward          *\n");
  printf("*    '¡ý' / 'S'  -- Move backward         *\n");
  printf("*    '¡û' / 'A'  -- Turn left             *\n");
  printf("*    '¡ú' / 'D'  -- Turn right            *\n");
  printf("*    'Q'         -- Yaw anticlockwise     *\n");
  printf("*    'E'         -- Yaw clockwise         *\n");
  printf("*    '+'         -- Move upward           *\n");
  printf("*    '-'         -- Move downward         *\n");
  printf("*                                         *\n");
  printf("* - Others -                              *\n");
  printf("*    'C'         -- Clear Position        *\n");
  printf("*    'P'         -- Print Position        *\n");
  printf("*    'F'         -- Find Colored Object   *\n");
  printf("*    'G'         -- Give Up Operations    *\n");
  printf("*    'Esc'       -- Exit                  *\n");
  printf("*                                         *\n");
  printf("*******************************************\n\n");

  while (1) {
    // Key input
    int key = cvWaitKey(20);
    if (key == 0x1b) break;

    // Update
    if (!ardrone.update()) break;

    // Reset the control vector
    vx = 0; vy = 0; vz = 0; vr = 0;

    // Accumulate the position vector
    ardrone.getVelocity(&vel_x, &vel_y, &vel_z);
    tick_time1 = clock();
    pos_x += vel_x * ((double)(tick_time1 - tick_time0)) / (double)(CLOCKS_PER_SEC);
    pos_y += vel_y * ((double)(tick_time1 - tick_time0)) / (double)(CLOCKS_PER_SEC);
    pos_z += vel_z * ((double)(tick_time1 - tick_time0)) / (double)(CLOCKS_PER_SEC);
    tick_time0 = tick_time1;

    // Inform the current position of the drone
    if (key == 'p' || key=='P') printf("pos = (%lf, %lf, %lf)\n", pos_x, pos_y, pos_z);
    
    // Get an new image
    if (image != NULL) cvReleaseImage(&image);
    image = cvCloneImage(ardrone.getImage());
    if (image_process != NULL) cvReleaseImage(&image_process);
    image_process = cvCreateImage(cvSize(image->width*scale, image->height*scale), image->depth, image->nChannels);
    cvResize(image, image_process, CV_INTER_NN);

    
    // Preprocess the image
    _ProcessImage(&image_process);

    // Find the colored object
    if (_FindColoredObject(image_process, expectedColor, colorThreshold, gradientThreshold, &delta_px, &delta_py))
    {
      // Display the result
      _DrawCrosshair(image_process, delta_px + image_process->width / 2, delta_py + image_process->height / 2, crosshairSize, crosshairColor);

      // Inform the screen position of the block
      //printf("Block on screen (%d, %d)\n", delta_px, delta_py);

      // Set the control vector
      if (autoFindObjectFlag)
      {
        // Locking above the block at center
        if (abs(delta_px) > image_process->width / 3) vy = (delta_px < 0) ? DEFAULT_MOVE_SPEED : -DEFAULT_MOVE_SPEED;
        if (abs(delta_py) > image_process->height / 3) vx = (delta_py < 0) ? DEFAULT_MOVE_SPEED : -DEFAULT_MOVE_SPEED;
        
        // Horizental up and down
        //if (abs(delta_py) > image_process->height / 4) vz = (delta_py < 0) ? DEFAULT_MOVE_SPEED : -DEFAULT_MOVE_SPEED;
        //vr = (delta_px < 0) ? DEFAULT_MOVE_SPEED : -DEFAULT_MOVE_SPEED;
      }

      // Check if block found at the central screen
      if (!blockFoundFlag && abs(delta_px) < image_process->width / 4 && abs(delta_py) < image_process->height / 4)
      {
        // Set the flag
        blockFoundFlag = true;

        // Inform the position
        printf("Block found at position (%lf, %lf)\n", pos_x, pos_y);
      }
    }
    
    // Command from keys
    switch (key)
    {
    case ' ':
      if (ardrone.onGround()) ardrone.takeoff();
      else                    ardrone.landing();
      break;

    case 0x260000: case 'w': case 'W': vx = DEFAULT_MOVE_SPEED; break;
    case 0x280000: case 's': case 'S': vx = -DEFAULT_MOVE_SPEED; break;
    case 0x250000: case 'a': case 'A': vy = DEFAULT_MOVE_SPEED; break;
    case 0x270000: case 'd': case 'D': vy = -DEFAULT_MOVE_SPEED; break;
    case 'q': case 'Q':                vr = DEFAULT_MOVE_SPEED; break;
    case 'e': case 'E':                vr = -DEFAULT_MOVE_SPEED; break;
    case '=':                          vz = DEFAULT_MOVE_SPEED; break;
    case '-':                          vz = -DEFAULT_MOVE_SPEED; break;

    case 'c': case 'C':
      pos_x = 0;
      pos_y = 0;
      pos_z = 0;
      break;

    case 'f': case 'F':
      autoFindObjectFlag = true;
      printf("Auto Find Object: Enabled.\n");
      break;

    case 'g': case 'G':
      autoFindObjectFlag = false;
      printf("Auto Find Object: Disabled.\n");
      break;
    }
    
    // Move
    ardrone.move3D(vx, vy, vz, vr);

    // Display the image
    cvShowImage("camera", image_process);
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
  //_MedianFilter(ptrImage);
  for (int i = 0; i < 1; ++i) _MedianFilter_OpenCV(ptrImage);
}

/**
* @param image The image obtain from the bottom camera.
* @param channelValues The value of each channel in expected color.
*                      Value of each channel ranges from 0 to 255.
*                      The numbers of channels must match that of the
*                      input image, otherwise memory leak may happen.
* @param colorThreshold The average tolerable error of the expected
*                       color.
* @param gradientThreshold The gradient threshold for DDSelector
*                          algorithm.
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
bool _FindColoredObject(IplImage* image, int* channelValues, int colorThreshold, int gradientThreshold, int* delta_px, int* delta_py)
{
  int pixelGroupSize = image->width * image->height;
  int* pixelGroup = new int[pixelGroupSize];
  int currentGroup;
  queue<int> toCheck;
  bool isExpectedPixel;
  int countGroupSize;
  int maxGroupSize;
  int tmpBlockTop, tmpBlockBottom, tmpBlockLeft, tmpBlockRight;


  // Check if parameters are valid
  for (int i = 0; i < image->nChannels; ++i)
  {
    if (channelValues[i]<0 || channelValues[i]>255) return false;
  }

  // Reset the pixel group
  for (int i = 0; i < pixelGroupSize; ++i) pixelGroup[i] = -1;

  // Check if the gradient threshold is valid
  if (gradientThreshold < 0) return false;

  // Loop check each pixel
  currentGroup = 0;
  maxGroupSize = 0;
  for (int i = 0; i < pixelGroupSize; ++i)
  {
    // Check if the pixel is within expected range
    isExpectedPixel = true;
    for (int channel = 0; channel < image->nChannels; ++channel)
    {
      if ((uchar)(image->imageData[i * image->nChannels + channel]) < channelValues[channel] - colorThreshold ||
        (uchar)(image->imageData[i * image->nChannels + channel]) > channelValues[channel] + colorThreshold)
      {
        isExpectedPixel = false;
        break;
      }
    }
    if (!isExpectedPixel) continue;

    // Unprocessed pixel
    countGroupSize = 0;
    tmpBlockTop = image->height - 1;
    tmpBlockBottom = 0;
    tmpBlockLeft = image->width - 1;
    tmpBlockRight = 0;
    if (pixelGroup[i] < 0)
    {
      // Reset to-check queue and pixel queue
      while (!toCheck.empty()) toCheck.pop();

      // Push this pixel to the queue
      toCheck.push(i);

      // Assign group number to target pixel
      pixelGroup[i] = currentGroup;

      // Loop find the valid gradient diffuse pixels
      while (!toCheck.empty())
      {
        int currentPixel = toCheck.front();

        // Check the block boundary
        int pixel_x = currentPixel % image->width;
        int pixel_y = currentPixel / image->width;
        if (pixel_x < tmpBlockLeft) tmpBlockLeft = pixel_x;
        if (pixel_x > tmpBlockRight) tmpBlockRight = pixel_x;
        if (pixel_y < tmpBlockTop) tmpBlockTop = pixel_y;
        if (pixel_y > tmpBlockBottom) tmpBlockBottom = pixel_y;

        // Add up the pixel count
        ++countGroupSize;

        // Check the neighbour pixels of to-check queue front
        _CheckNeighbourPixels(image, pixelGroup, currentGroup, currentPixel, toCheck, gradientThreshold);

        // Pop the processed pixel from the to-check queue
        toCheck.pop();
      }

      // Check if it is the largest block
      if (countGroupSize > maxGroupSize)
      {
        // Set the max group size
        maxGroupSize = countGroupSize;

        // Set the location of the block referred to the screen center
        (*delta_px) = (tmpBlockLeft + tmpBlockRight - image->width) / 2;
        (*delta_py) = (tmpBlockTop + tmpBlockBottom - image->height) / 2;
      }

      // Move to the next pixel group
      ++currentGroup;
    }
  }

  // Release resources
  delete pixelGroup;

  // Check block exist
  return (maxGroupSize > 100);
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
        arr[i][j] = (uchar)(image->imageData[(currentBlock - image->width - 1 + i) * 3 + j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < image->nChannels; ++j) {
        arr[i + 3][j] = (uchar)(image->imageData[(currentBlock - 1 + i) * 3 + j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < image->nChannels; ++j) {
        arr[i + 6][j] = (uchar)(image->imageData[(currentBlock + image->width - 1 + i) * 3 + j]);
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
    for (int i = 0; i < 3; ++i)
    {
      image_dst->imageData[currentBlock * 3 + i] = arr[5][i];
    }
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
        if (abs(((uchar)image->imageData[currentPixel * image->nChannels + channel]) - (uchar)(image->imageData[targetPixel * image->nChannels + channel])) > threshold)
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
        if (abs((uchar)(image->imageData[currentPixel * image->nChannels + channel]) - (uchar)(image->imageData[targetPixel * image->nChannels + channel])) > threshold)
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
        if (abs((uchar)(image->imageData[currentPixel * image->nChannels + channel]) - (uchar)(image->imageData[targetPixel * image->nChannels + channel])) > threshold)
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
        if (abs((uchar)(image->imageData[currentPixel * image->nChannels + channel]) - (uchar)(image->imageData[targetPixel * image->nChannels + channel])) > threshold)
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

/**
* @param image The image to add a crosshair to.
* @param x The coordinate in pixel referred to the left boundary.
* @param y The coordinate in pixel referred to the right boundary.
* @param size The size of the crosshair. It should be greater than 3.
* @param color The color of the crosshair. It should match the number
*              of channels of the image, otherwise memory leak may
*              happen. Each element ranges from 0 to 255.
*
* @brief
*    Draw a crosshair in specific location on a image.
*/
void _DrawCrosshair(IplImage* image, int x, int y, int size, int* color)
{
  int halfSize;
  int leftBoundary, rightBoundary, topBoundary, bottomBoundary;

  // Check the parameters
  if (x<0 || x>image->width - 1) return;
  if (y<0 || y>image->height - 1) return;
  if (size < 3) return;

  // Calculate the derived parameters
  halfSize = (size - 1) / 2;
  leftBoundary = x - halfSize;
  rightBoundary = x + halfSize;
  topBoundary = y - halfSize;
  bottomBoundary = y + halfSize;

  // Draw the horizontal line of the crosshair
  for (int i = leftBoundary; i <= rightBoundary; ++i)
  {
    // Check image boundary
    if (i >= 0 && i < image->width)
    {
      // Draw the valid pixels
      for (int channel = 0; channel < image->nChannels; ++channel)
      {
        image->imageData[(y*image->width + i)*image->nChannels + channel] = color[channel];
      }
    }
  }

  // Draw the vertical line of the crosshair
  for (int i = topBoundary; i <= bottomBoundary; ++i)
  {
    // Check image boundary
    if (i >= 0 && i < image->height)
    {
      // Draw the valid pixels
      for (int channel = 0; channel < image->nChannels; ++channel)
      {
        image->imageData[(i*image->width + x)*image->nChannels + channel] = color[channel];
      }
    }
  }
}

#endif //_LOCATE_COLORED_OBJECT_HPP_
