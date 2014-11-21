#include "motion_detection.h"

MotionDetection::MotionDetection()
{
    ros::NodeHandle n;
    ros::NodeHandle p_n("~"); //private nh

    img_prev_ptr_.reset();
    img_current_ptr_.reset();

    image_transport::ImageTransport it(n);
    image_transport::ImageTransport image_motion_it(p_n);
    image_transport::ImageTransport image_detected_it(p_n);

    //camera_sub_ = it.subscribeCamera("/rgb/image_color", 100, &MotionDetection::imageCallback, this);

    image_sub_ = it.subscribe("/camera/rgb/image_raw", 1, &MotionDetection::imageCallback, this);

    dyn_rec_server_.setCallback(boost::bind(&MotionDetection::dynRecParamCallback, this, _1, _2));

    image_percept_pub_ = n.advertise<hector_worldmodel_msgs::ImagePercept>("image_percept", 100);
    image_motion_pub_ = image_motion_it.advertiseCamera("image_motion", 1);
    image_detected_pub_ = image_detected_it.advertiseCamera("image_detected", 1);
    cv::namedWindow("view");
}

MotionDetection::~MotionDetection() {
 //cv::destroyAllWindows();
   cv::destroyWindow("view");
}


void MotionDetection::imageCallback(const sensor_msgs::ImageConstPtr& img) //, const sensor_msgs::CameraInfoConstPtr& info)
{
  
  // get image
  cv_bridge::CvImageConstPtr img_next_ptr(cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8));
    
  if (img_prev_ptr_)
  {
    // Calc differences between the images and do AND-operation
    // threshold image, low differences are ignored (ex. contrast change due to sunlight)
    cv::Mat diff1;
    cv::Mat diff2;
    cv::Mat img_motion;
    // d1 and d2 for calculating the differences
    // result, the result of and operation, calculated on d1 and d2
    // number_of_changes, the amount of changes in the result matrix.
    //cv::Mat d1, d2, motion;
    cv::absdiff(img_prev_ptr_->image, img_next_ptr->image, diff1);
    cv::absdiff(img_current_ptr_->image, img_next_ptr->image, diff2);
    cv::bitwise_and(diff1, diff2, img_motion);
    cv::threshold(img_motion, img_motion, motion_detect_threshold_, 255, CV_THRESH_BINARY);
    cv::Mat kernel_ero = getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    cv::erode(img_motion, img_motion, kernel_ero);
    //printf("%d\n", (int)Kernel_ero);
    cv::imshow("Erode", kernel_ero);
    cv::imshow("Motion", diff2);
    //cv::saveImg(diff2,DIR,EXT,DIR_FORMAT.c_str(),FILE_FORMAT.c_str());
   // std::string file_region = "/home/vibek/hector_motion_detection/region.xml";
    //std::vector<cv::Point2f> region;
   // cv::Point2f region;
     // If more than 'there_is_motion' pixels are changed, we say there is motion
    // and store an image on disk
   // int there_is_motion = 5;
    
    // Maximum deviation of the image, the higher the value, the more motion is allowed
   // int max_deviation = 100;
    
    unsigned int number_of_changes = 0;
    int min_x = img_motion.cols, max_x = 0;
    int min_y = img_motion.rows, max_y = 0;
    
    // loop over image and detect changes
    for(int j = 0; j < img_motion.rows; j++) { // height
      for(int i = 0; i < img_motion.cols; i++) { // width
        // check if at pixel (j,i) intensity is equal to 255
        // this means that the pixel is different in the sequence
        // of images (prev_frame, current_frame, next_frame)
        if(static_cast<int>(img_motion.at<uchar>(j,i)) == 255) {
          number_of_changes++;
          if (min_x > i) min_x = i;
          if (max_x < i) max_x = i;
          if (min_y > j) min_y = j;
          if (max_y < j) max_y = j;
        }
      }
    }
   //Printf("%d, %d", min_x,min_y);
    cv::Mat img_detected;
    img_current_col_ptr_->image.copyTo(img_detected);

    double percept_size = std::max(std::abs(max_x-min_x), std::abs(max_y-min_y));
    double area = std::abs(max_x-min_x) * std::abs(max_y-min_y);
    double density = area > 0.0 ? number_of_changes/area : 0.0;
    printf("Area: %lf,Density:%lf\n:,Percept:%lf\n: ",area, density, percept_size);
   

	ROS_INFO(">>>>>>>>>May be i will draw something");
   // if (number_of_changes && max_percept_size > percept_size && percept_size > min_percept_size && density > min_density)
   // {
      cv::rectangle(img_detected, cv::Rect(cv::Point(min_x, min_y), cv::Point(max_x, max_y)), CV_RGB(255,0,0), 5);
	ROS_INFO(">>>>>>>>>I'm drawing");
   // }

    cv::imshow("view", img_current_ptr_->image);
     
    //cv::imshow("view_detecde", image_current_col_ptr_->img_detected);
    cv::waitKey(3);
    sensor_msgs::CameraInfo::Ptr info;
    info.reset(new sensor_msgs::CameraInfo());
    info->header = img->header;

    if(image_motion_pub_.getNumSubscribers() > 0)
    {
      cv_bridge::CvImage cvImg;
      //img_motion.copyTo(cvImg.image);
      cvImg.header = img->header;
      cvImg.image = img_motion;
      cvImg.encoding = sensor_msgs::image_encodings::MONO8;
      image_motion_pub_.publish(cvImg.toImageMsg(), info);
    }

    if(image_detected_pub_.getNumSubscribers() > 0)
    {
      cv_bridge::CvImage cvImg;
      //img_detected.copyTo(cvImg.image);
      cvImg.header = img->header;
      cvImg.image = img_detected;
      cvImg.encoding = sensor_msgs::image_encodings::BGR8;
      image_detected_pub_.publish(cvImg.toImageMsg(), info);
    }
  }

  // shift image buffers
  img_prev_ptr_= img_current_ptr_;
  img_current_ptr_ = img_next_ptr;

  img_current_col_ptr_ = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);


//  // calculate the standard deviation
//  Scalar mean, stddev;
//  meanStdDev(motion, mean, stddev);

//  // if not to much changes then the motion is real (neglect agressive snow, temporary sunlight)
//  if(stddev[0] < max_deviation)
//  {
//    int number_of_changes = 0;
//    int min_x = motion.cols, max_x = 0;
//    int min_y = motion.rows, max_y = 0;
//    // loop over image and detect changes
//    for(int j = y_start; j < y_stop; j+=2) { // height
//      for(int i = x_start; i < x_stop; i+=2) { // width
//        // check if at pixel (j,i) intensity is equal to 255
//        // this means that the pixel is different in the sequence
//        // of images (prev_frame, current_frame, next_frame)
//        if(static_cast<int>(motion.at<uchar>(j,i)) == 255) {
//          number_of_changes++;
//          if(min_x>i) min_x = i;
//          if(max_x<i) max_x = i;
//          if(min_y>j) min_y = j;
//          if(max_y<j) max_y = j;
//        }
//      }
//    }

//    if(number_of_changes) {
//      //check if not out of bounds
//      if(min_x-10 > 0) min_x -= 10;
//      if(min_y-10 > 0) min_y -= 10;
//      if(max_x+10 < result.cols-1) max_x += 10;
//      if(max_y+10 < result.rows-1) max_y += 10;
//      // draw rectangle round the changed pixel
//      Point x(min_x,min_y);
//      Point y(max_x,max_y);
//      Rect rect(x,y);
//      Mat cropped = result(rect);
//      cropped.copyTo(result_cropped);
//      rectangle(result,rect,color,1);
//    }
//    //return number_of_changes;
//  }
  //return 0;
}

/*void MotionDetection::parseRegionXML(std::string file_region, std::vector<Point2f> &region)
{
 TiXmlDocument doc(file_region.c_str());
if(doc.LoadFile()) // ok file loaded correctly
{
TiXmlElement * point = doc.FirstChildElement("point");
int x, y;
while (point)
{
point->Attribute("x",&x);
point->Attribute("y",&y);
Point2f p(x,y);
region.push_back(p);
point = point->NextSiblingElement("point");
}
}
else
exit(1);
}*/

//void MotionDetection::mappingCallback(const thermaleye_msgs::Mapping& mapping)
//{
//   mapping_ = mapping;
//   mappingDefined_ = true;
//   ROS_INFO("Mapping received");
//}

void MotionDetection::dynRecParamCallback(MotionDetectionConfig &config, uint32_t level)
{
  motion_detect_threshold_ = config.motion_detect_threshold;
  min_percept_size = config.motion_detect_min_percept_size;
  max_percept_size = config.motion_detect_max_percept_size;
  min_density = config.motion_detect_min_density;
  percept_class_id_ = config.percept_class_id;
}

int main(int argc, char **argv)
{
  
  //cv::startWindowThread();

  ros::init(argc, argv, "motion_detection");
  MotionDetection md;
  ros::spin();
  
  return 0;
}

