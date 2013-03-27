#include "cv.h"
#include "cxmisc.h"
#include "highgui.h"
#include "cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace std;

int main(int argc, char *argv[])
{
    if (argc != 5) {
        //printf("%d", argc);
        printf("Please use: ./rectify [left] [right] [left_out] [right_out]\n");
	return 1;
    }

    //Load left, left image, and rightbig, the unresized right image

    IplImage* left=cvLoadImage(argv[1],0);
    IplImage* rightbig=cvLoadImage(argv[2],0);

    CvSize imageSize = cvGetSize(left);


    //Resize right image to be equivilent to left
    IplImage * right = cvCreateImage(imageSize, rightbig->depth, rightbig->nChannels);
    cvResize(rightbig, right);

    CvMat* mx1 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* my1 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* mx2 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* my2 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );

    CvMat* leftr = cvCreateMat( imageSize.height,
        imageSize.width, CV_8U );
    CvMat* rightr = cvCreateMat( imageSize.height,
        imageSize.width, CV_8U );


    //Use hardcoded rectify matrices
    //Note -- these are calibrated to the left and right camera of the Dragonboard
    mx1 = (CvMat *) cvLoad("mx1.xml");
    my1 = (CvMat *) cvLoad("my1.xml");
    mx2 = (CvMat *) cvLoad("mx2.xml");
    my2 = (CvMat *) cvLoad("my2.xml");


    //rectify images
    cvRemap( left, leftr, mx1, my1 );
    cvRemap( right, rightr, mx2, my2 );	


    //save rectified images
    cvSaveImage(argv[3], leftr);
    cvSaveImage(argv[4], rightr);

    

    

    return 0;
}
