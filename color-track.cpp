
#include <iostream>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.h"
#include "opencv/cxmisc.h"
#include <iomanip>
#include <math.h>
#include <string.h>
#include <time.h>
#include "pololu_servo.h"

#include <stdio.h>
using namespace std;

// Various color types for detected shirt colors.
enum
{
    cBLACK = 0,
    cWHITE,
    cGREY,
    cRED,
    cORANGE,
    cYELLOW,
    cGREEN,
    cAQUA,
    cBLUE,
    cPURPLE,
    cPINK,
    NUM_COLOR_TYPES
};
const char* sCTypes[NUM_COLOR_TYPES] = { "Black", "White", "Grey", "Red", "Orange", "Yellow", "Green", "Aqua", "Blue", "Purple", "Pink" };
uchar cCTHue[NUM_COLOR_TYPES] = { 0, 0, 0, 0, 20, 30, 55, 85, 115, 138, 161 };
uchar cCTSat[NUM_COLOR_TYPES] = { 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255 };
uchar cCTVal[NUM_COLOR_TYPES] = { 0, 255, 120, 255, 255, 255, 255, 255, 255, 255, 255 };

int getPixelColorType(int H, int S, int V)
{
    int color;
    if (V < 75)
        color = cBLACK;
    else if (V > 190 && S < 27)
        color = cWHITE;
    else if (S < 53 && V < 185)
        color = cGREY;
    else
    { // Is a color
        if (H < 14)
            color = cRED;
        else if (H < 25)
            color = cORANGE;
        else if (H < 34)
            color = cYELLOW;
        else if (H < 73)
            color = cGREEN;
        else if (H < 102)
            color = cAQUA;
        else if (H < 127)
            color = cBLUE;
        else if (H < 149)
            color = cPURPLE;
        else if (H < 175)
            color = cPINK;
        else
            // full circle
            color = cRED; // back to Red
    }
    return color;
}

IplImage* cropRectangle(IplImage *img, CvRect region)
{
    IplImage *imageTmp, *imageRGB;
    CvSize size;
    size.height = img->height;
    size.width = img->width;

    if (region.width <= 0)
        region.width = 1;
    if (region.height <= 0)
        region.height = 1;
    if (region.x <= 0)
        region.x = 1;
    if (region.y <= 0)
        region.y = 1;
    if (region.x > 640 - 21)
        region.x = 640 - 21;
    if (region.y > 480 - 21)
        region.y = 480 - 21;

    //cout <<region.x<<", "<<region.y<<", "<<region.height<<", "<<region.width<<endl;
    if (img->depth != IPL_DEPTH_8U)
    {
        std::cerr << "ERROR: Unknown image depth of " << img->depth << " given in cropRectangle() instead of 8." << std::endl;
        exit(1);
    }

    // First create a new (color or greyscale) IPL Image and copy contents of img into it.
    imageTmp = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
    cvCopy(img, imageTmp);

    // Create a new image of the detected region
    //printf("Cropping image at x = %d, y = %d...", faces[i].x, faces[i].y);
    //printf("Setting region of interest...");
    // Set region of interest to that surrounding the face

    cvSetImageROI(imageTmp, region);
    // Copy region of interest (i.e. face) into a new iplImage (imageRGB) and return it
    size.width = region.width;
    size.height = region.height;
    imageRGB = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
    cvCopy(imageTmp, imageRGB); // Copy just the region.

    cvReleaseImage(&imageTmp);
    return imageRGB;
}

IplImage* GetThresholdedImage(IplImage* img)
{
// Convert the image into an HSV image
    CvScalar hsv_min, hsv_max, hsv_min2, hsv_max2;
    hsv_min = cvScalar(100, 120, 80);
    hsv_max = cvScalar(127, 255, 255);
    hsv_min2 = cvScalar(1, 120, 80);
    hsv_max2 = cvScalar(10, 255, 255);
    IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
    cvCvtColor(img, imgHSV, CV_BGR2HSV);
    IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);
    IplImage* imgThreshed2 = cvCreateImage(cvGetSize(img), 8, 1);

// Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm

    //cvInRangeS(imgHSV, cvScalar(20, 100, 100), cvScalar(30, 255, 255),imgThreshed);
    cvInRangeS(imgHSV, hsv_min, hsv_max, imgThreshed);
    //cvInRangeS(imgHSV, hsv_min2, hsv_max2, imgThreshed2);
    //cvOr(imgThreshed, imgThreshed2, imgThreshed);

    cvReleaseImage(&imgHSV);
    cvReleaseImage(&imgThreshed2);

    return imgThreshed;
}
int main(int, char**)
{

    //servo variables
    const char *portName;
    int baudRate;
    unsigned short targetx = 1350, targety = 1350;
    float errorx, errory, kpy = 0.3, kpx = 0.3;

    portName = "/dev/ttyACM0";
    baudRate = 9600;

    /* Open the Maestro's serial port. */
    int fd = openPort(portName, baudRate);

    CvRect rect = { 1, 1, 1, 1 };
    CvCapture* capture = 0;
    capture = cvCaptureFromCAM(0);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
    //cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, 5);

    // Couldn't get a device? Throw an error and quit
    if (!capture)
    {
        printf("Could not initialize capturing...\n");
        return -1;
    }

    // The two windows we'll be using
    cvNamedWindow("video");
    //cvNamedWindow("thresh");

    // This image holds the "scribble" data...
    // the tracked positions of the ball
    IplImage* imgScribble = NULL;
    //IplImage* contourImg = NULL;

    // An infinite loop
    while (true)
    {
        //Sleep(500);
        // Will hold a frame captured from the camera
        //double fps = cvGetCaptureProperty(capture,CV_CAP_PROP_FPS);
        //cout <<"fps: "<< fps<<endl;
        IplImage* frame = 0;
        frame = cvQueryFrame(capture);		//freenect_sync_get_rgb_cv(0);//cvQueryFrame(capture);
        //cvCvtColor(frame,frame, CV_RGB2BGR);

        // If we couldn't grab a frame... quit
        if (!frame)
            break;

        // If this is the first frame, we need to initialize it
        if (imgScribble == NULL)
        {
            imgScribble = cvCreateImage(cvGetSize(frame), 8, 3);
        }

        // Holds the yellow thresholded image (yellow = white, rest = black)
        CvSeq* contours;
        int mode = CV_RETR_EXTERNAL;
        CvMemStorage *storage = cvCreateMemStorage(0);
        IplImage* imgYellowThresh = GetThresholdedImage(frame);
        //cvSmooth(imgYellowThresh, imgYellowThresh, CV_GAUSSIAN, 9, 9);
        //cvThreshold(imgYellowThresh, imgYellowThresh, 25, 255, CV_THRESH_BINARY);
        //contourImg = cvCloneImage(imgYellowThresh);
        //cvNamedWindow("Contour", 1);
        //find the contour
        cvFindContours(imgYellowThresh, storage, &contours, sizeof(CvContour), mode, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
        int i = 0;
        //findContours( imgYellowThresh, contours1, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        for (; contours != 0; contours = contours->h_next)
        {
            i++;
            //ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 ); //randomly coloring different contours
            rect = cvBoundingRect(contours, 0);
            if (rect.height > 10 && rect.width > 10)
            {
                cvRectangle(frame, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height), cvScalar(0, 0, 255, 0), 1,
                    8, 0);
            }
            //cvDrawContours(frame, contours, CV_RGB(0, 255, 0),CV_RGB(255, 0, 0), 2, 2, 8, cvPoint(0, 0));
            //float* p = (float*) cvGetSeqElem(contours, i);
            //cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])), 3,CV_RGB(0,255,0), -1, 8, 0);
            //cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])),cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0);
        }

        //printf("Total Contours:%d\n", i);
        /*for (int j = 0; j < i; j++) {

         float* p = (float*) cvGetSeqElem(contours, j);
         cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])), 3,CV_RGB(0,255,0), -1, 8, 0);
         cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])),cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0);
         }*/
        //cvShowImage("Contour", contourImg);
        // Calculate the moments to estimate the position of the ball
        CvMoments *moments = (CvMoments*) malloc(sizeof(CvMoments));
        cvMoments(imgYellowThresh, moments, 1);

        // The actual moment values
        double moment10 = cvGetSpatialMoment(moments, 1, 0);
        double moment01 = cvGetSpatialMoment(moments, 0, 1);
        double area = cvGetCentralMoment(moments, 0, 0);

        // Holding the last and current ball positions
        static int posX = 0;
        static int posY = 0;

        int lastX = posX;
        int lastY = posY;

        posX = moment10 / area;
        posY = moment01 / area;

        // Print it out for debugging purposes
        //int area1 = (int)area;
        //printf("position %d,%d", posX, posY);

        //cout << "X: " << posX << "\tY: " << posY << "\tarea: " << area<< "\tradius: " << endl;

        //printf("position (%d,%d,%f)",posX,posY,area);

        // We want to draw a line only if its a valid position
        if (lastX > 0 && lastY > 0 && posX > 0 && posY > 0)
        {
            // Draw a yellow line from the previous point to the current point
            //cvLine(imgScribble, cvPoint(posX, posY), cvPoint(lastX, lastY),cvScalar(0, 255, 255), 5);
        }

        // Add the scribbling image and the frame... and we get a combination of the two
        cvRectangle(frame, cvPoint(posX - 10, posY - 10), cvPoint(posX + 10, posY + 10), CV_RGB(0, 0, 255), 2, 8, 0);
        //calculate error and turn the servo
        errorx = 320 - posX;
        errory = 240 - posY;
        //cout << errorx;
        //angle=angle*10.25+505;
        targetx += errorx * kpx;
        targety += errory * kpy;
        //if(flag)
        //maestroSetTarget(fd, 0, targetx*4);
        //maestroSetTarget(fd, 1, targety*4);

        //	cvRectangle(frame, cvPoint(rect.x + rect.width / 4, rect.y + rect.height),cvPoint(rect.x + 10 + rect.width / 4,	rect.y + rect.height + 10), CV_RGB(255,255,255), 2, 8,0);

        // Convert the shirt region from RGB colors to HSV colors
        //cout << "Converting shirt region to HSV" << endl;
        if (rect.x <= 0)
            rect.x = 1;
        if (rect.y <= 0)
            rect.y = 1;

        CvRect iArea = cvRect(rect.x + rect.width / 4, rect.y + rect.height, 10, 20);

        IplImage *imageShirt = cropRectangle(frame, iArea); //cvRect((int)(rect.x+rect.width/2),(int)(rect.y+rect.height),10,10));
        IplImage *imageShirtHSV = cvCreateImage(cvGetSize(imageShirt), 8, 3);
        //IplImage* imgAreaHSV = cvCreateImage(cvGetSize(imageShirt), 8, 3);
        //cvCvtColor(imageShirt, imgAreaHSV, CV_BGR2HSV);
        //cvShowImage("thresh", imageShirt);
        cvCvtColor(imageShirt, imageShirtHSV, CV_BGR2HSV); // (note that OpenCV stores RGB images in B,G,R order.
        if (!imageShirtHSV)
        {
            cerr << "ERROR: Couldn't convert Shirt image from BGR2HSV." << endl;
            exit(1);
        }

        //cout << "Determining color type of the shirt" << endl;
        int h = imageShirtHSV->height; // Pixel height
        int w = imageShirtHSV->width; // Pixel width
        int rowSize = imageShirtHSV->widthStep; // Size of row in bytes, including extra padding
        char *imOfs = imageShirtHSV->imageData; // Pointer to the start of the image HSV pixels.
        // Create an empty tally of pixel counts for each color type
        int tallyColors[NUM_COLOR_TYPES];
        for (int i = 0; i < NUM_COLOR_TYPES; i++)
            tallyColors[i] = 0;
        // Scan the shirt image to find the tally of pixel colors
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                // Get the HSV pixel components
                uchar H = *(uchar*) (imOfs + y * rowSize + x * 3 + 0); // Hue
                uchar S = *(uchar*) (imOfs + y * rowSize + x * 3 + 1); // Saturation
                uchar V = *(uchar*) (imOfs + y * rowSize + x * 3 + 2); // Value (Brightness)

                // Determine what type of color the HSV pixel is.
                int ctype = getPixelColorType(H, S, V);
                // Keep count of these colors.
                tallyColors[ctype]++;
            }
        }

        // Print a report about color types, and find the max tally
        //cout << "Number of pixels found using each color type (out of " << (w*h) << ":\n";
        int tallyMaxIndex = 0;
        int tallyMaxCount = -1;
        int pixels = w * h;
        for (int i = 0; i < NUM_COLOR_TYPES; i++)
        {
            int v = tallyColors[i];
            if ((v * 100 / pixels) > 50)
                //cout << sCTypes[i] << " " << (v * 100 / pixels) << "%, ";

                if (v > tallyMaxCount)
                {
                    tallyMaxCount = tallyColors[i];
                    tallyMaxIndex = i;
                }
        }
        //int percentage = initialConfidence * (tallyMaxCount * 100 / pixels);
        //cout << "Color of shirt: " << sCTypes[tallyMaxIndex] << " (" << percentage << "% confidence)." << endl << endl;

        // Display the color type over the shirt in the image.
        //cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,0.55,0.7, 0,1,CV_AA);	// For OpenCV 1.1
        //cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,0.8,1.0, 0,1, CV_AA);	// For OpenCV 2.0
        //sprintf_s(text, sizeof(text)-1, "%d%%", percentage);
        //cvPutText(frame, sCTypes[tallyMaxIndex], cvPoint(rect.x, rect.y + rect.height + 12), &font, CV_RGB(255,0,0));
        //cvPutText(frame, text, cvPoint(rect.x, rect.y + rect.height + 24), &font, CV_RGB(255,0,0));

        // Free resources.
        cvReleaseImage(&imageShirtHSV);
        cvReleaseImage(&imageShirt);

        //cout << "X: " << posX << "\tY: " << posY << "\tarea: " << area<< "\tradius: " << endl;
        cvAdd(frame, imgScribble, frame);
        //cvShowImage("thresh", imgYellowThresh);
        cvShowImage("video", frame);

        // Wait for a keypress
        int c = cvWaitKey(10);
        if (c != -1)
        {
            // If pressed, break out of the loop
            break;
        }

        // Release the thresholded image... we need no memory leaks.. please
        cvReleaseImage(&imgYellowThresh);

        delete moments;

    }
    // We're done using the camera. Other applications can now use it
    cvReleaseCapture(&capture);
    closePort(fd);
    return 0;
}

IplImage *GlViewColor(IplImage *depth)
{
    static IplImage *image = 0;
    if (!image)
        image = cvCreateImage(cvSize(640, 480), 8, 3);
    unsigned char *depth_mid = (unsigned char*) (image->imageData);
    int i;
    for (i = 0; i < 640 * 480; i++)
    {
        int lb = ((short *) depth->imageData)[i] % 256;
        int ub = ((short *) depth->imageData)[i] / 256;
        switch (ub)
        {
        case 0:
            depth_mid[3 * i + 2] = 255;
            depth_mid[3 * i + 1] = 255 - lb;
            depth_mid[3 * i + 0] = 255 - lb;
            break;
        case 1:
            depth_mid[3 * i + 2] = 255;
            depth_mid[3 * i + 1] = lb;
            depth_mid[3 * i + 0] = 0;
            break;
        case 2:
            depth_mid[3 * i + 2] = 255 - lb;
            depth_mid[3 * i + 1] = 255;
            depth_mid[3 * i + 0] = 0;
            break;
        case 3:
            depth_mid[3 * i + 2] = 0;
            depth_mid[3 * i + 1] = 255;
            depth_mid[3 * i + 0] = lb;
            break;
        case 4:
            depth_mid[3 * i + 2] = 0;
            depth_mid[3 * i + 1] = 255 - lb;
            depth_mid[3 * i + 0] = 255;
            break;
        case 5:
            depth_mid[3 * i + 2] = 0;
            depth_mid[3 * i + 1] = 0;
            depth_mid[3 * i + 0] = 255 - lb;
            break;
        default:
            depth_mid[3 * i + 2] = 0;
            depth_mid[3 * i + 1] = 0;
            depth_mid[3 * i + 0] = 0;
            break;
        }
    }
    return image;
}

/*int main(int argc, char **argv)
 {
 while (cvWaitKey(10) < 0) {
 IplImage *image = freenect_sync_get_rgb_cv(0);
 if (!image) {
 printf("Error: Kinect not connected?\n");
 return -1;
 }
 cvCvtColor(image, image, CV_RGB2BGR);
 IplImage *depth = freenect_sync_get_depth_cv(0);
 if (!depth) {
 printf("Error: Kinect not connected?\n");
 return -1;
 }
 cvShowImage("RGB", image);
 cvShowImage("Depth", GlViewColor(depth));
 }
 return 0;
 }*/
