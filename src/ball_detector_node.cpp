#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;


class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    int iLowH;
    int iHighH;

    int iLowS;
    int iHighS;

    int iLowV;
    int iHighV;
    Mat imgLines;
    int iLastX;
    int iLastY;
public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

        iLowH = 0;
        iHighH = 12;

        iLowS = 100;
        iHighS = 220;

        iLowV = 190;
        iHighV = 255;
        iLastX = -1;
        iLastY = -1;
        //Create trackbars in "Control" window
        cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
        cvCreateTrackbar("HighH", "Control", &iHighH, 179);

        cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", "Control", &iHighS, 255);

        cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", "Control", &iHighV, 255);
        //Create a black image with the size as the camera output
        //imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }


        Mat imgHSV;

        cv::cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        cv::Mat imgThresholded;

        cv::inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );


        //Calculate the moments of the thresholded image
//        Moments oMoments = moments(imgThresholded);

//        double dM01 = oMoments.m01;
//        double dM10 = oMoments.m10;
//        double dArea = oMoments.m00;

//        // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
//        if (dArea > 10000)
//        {
//            //calculate the position of the ball
//            int posX = dM10 / dArea;
//            int posY = dM01 / dArea;

//            if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
//            {
//                //Draw a red line from the previous point to the current point
//                line(cv_ptr->image, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
//            }

//            iLastX = posX;
//            iLastY = posY;
//        }
        using namespace cv;
        // Setup SimpleBlobDetector parameters.
        SimpleBlobDetector::Params params;

        // Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 255;

        params.filterByColor=true;
        params.blobColor=255;
        // Filter by Area.
        params.filterByArea = true;
        params.minArea = 150;
        params.maxArea =20000;

        // Filter by Circularity
        params.filterByCircularity = false;
        params.minCircularity = 0.1;

        // Filter by Convexity
        params.filterByConvexity = false;
        params.minConvexity = 0.87;

        // Filter by Inertia
        params.filterByInertia = false;
        params.minInertiaRatio = 0.01;
        // Set up the detector with default parameters.
        cv::SimpleBlobDetector detector(params);

        // Detect blobs.
        std::vector<KeyPoint> keypoints;
        detector.detect( imgThresholded, keypoints);

        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob

        drawKeypoints( imgThresholded, keypoints, imgThresholded, Scalar(255,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );



        imshow("Thresholded Image", imgThresholded); //show the thresholded image
        //cv_ptr->image = cv_ptr->image + imgLines;
        imshow("Original", cv_ptr->image); //show the original image

        // if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        // {
        //     ROS_INFO("esc key is pressed by user");
        //     break;
        // }

        // Draw an example circle on the video stream
        // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

        // Update GUI Window
        //  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
