#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_client.hh>

#include <arduplanner.h>

std::shared_ptr<ArduPlanner> o_arduplanner;
int count = 0;
// Function is called everytime an image message is received from Gazebo
void imageCb(ConstImagesStampedPtr &msg)
{
  if(o_arduplanner->is_cloud_processed.load())
  {
    o_arduplanner->updateSensorToWorld();
    int width;
    int height;
    char *data_l;
    char *data_r;

    width = (int) msg->image(0).width();
    height = (int) msg->image(0).height();
    data_l = new char[msg->image(0).data().length() + 1];
    data_r = new char[msg->image(1).data().length() + 1];

    memcpy(data_l, msg->image(0).data().c_str(), msg->image(0).data().length());
    cv::Mat image_l(height, width, CV_8UC3, data_l);

    memcpy(data_r, msg->image(1).data().c_str(), msg->image(1).data().length());
    cv::Mat image_r(height, width, CV_8UC3, data_r);
    cv::cvtColor(image_l, image_l, CV_BGR2RGB);
    cv::cvtColor(image_r, image_r, CV_BGR2RGB);

    auto startTime = std::chrono::system_clock::now();
    
    cv::Mat disparity = o_arduplanner->o_stereo->matchPair(image_l,image_r);
    
    std::chrono::duration<double> total_elapsed = std::chrono::system_clock::now() - startTime;
    // DBG("Disparity computation time: " << total_elapsed.count());
    
    cv::imshow("disparity", disparity);

    cv::waitKey(10);
    delete data_l;
    delete data_r;
    if(std::abs(o_arduplanner->sensorToWorld(2,3)) > 1e-2)
    {
      if(count < 90)
        o_arduplanner->initializeManeuver(count * 4 * 3.14/180.0);
      else
      {
        o_arduplanner->processCloudThread();
      }
      count++;
    }
    else
      DBG("Mavlink not initialized, current height " << std::abs(o_arduplanner->sensorToWorld(2,3)));
  }
}

int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  o_arduplanner = std::make_shared<ArduPlanner>();
  // Listen to Gazebo topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/iris/iris_demo/cam_link/stereo_camera/images", imageCb);

  // Busy wait loop
  while (true)
  {  
    gazebo::common::Time::MSleep(10);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}