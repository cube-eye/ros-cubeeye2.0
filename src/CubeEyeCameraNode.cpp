#include <sstream>
#include <thread>
#include <signal.h>

#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

// service
#include "cubeeye_camera/LastState.h"
#include "cubeeye_camera/LastError.h"
#include "cubeeye_camera/Scan.h"
#include "cubeeye_camera/Connect.h"
#include "cubeeye_camera/Run.h"
#include "cubeeye_camera/Stop.h"
#include "cubeeye_camera/Disconnect.h"

#include "ProjectDefines.h"
#include "CameraModule.h"
#include "CubeEyeCameraNode.h"

#if 1
#include <dynamic_reconfigure/server.h>
#include <cubeeye_camera/cubeeye_scubeConfig.h>
#include <cubeeye_camera/cubeeye_icubeConfig.h>
#endif

std::shared_ptr<CubeEyeCameraNode> CubeEyeCameraNode::mInstance;

CubeEyeCameraNode::CubeEyeCameraNode() : mCamera(std::make_shared<CameraModule>()) 
{
}

std::shared_ptr<CubeEyeCameraNode> CubeEyeCameraNode::getInstance() 
{
    if (!mInstance) {
        mInstance = std::make_shared<CubeEyeCameraNode>();
    }
    return mInstance;
}

void CubeEyeCameraNode::init(ros::NodeHandle node)
{
    mNode = node;

    // init services
    mLastStateService = mNode.advertiseService("/cubeeye_camera_node/get_last_state", 
                            &CubeEyeCameraNode::getLastStateServiceCallback, this);
    mLastErrorService = mNode.advertiseService("/cubeeye_camera_node/get_last_error", 
                            &CubeEyeCameraNode::getLastErrorServiceCallback, this);
    mScanService = mNode.advertiseService("/cubeeye_camera_node/scan", 
                            &CubeEyeCameraNode::getScanServiceCallback, this);
    mConnectService = mNode.advertiseService("/cubeeye_camera_node/connect", 
                            &CubeEyeCameraNode::getConnectServiceCallback, this);
    mRunService = mNode.advertiseService("/cubeeye_camera_node/run", 
                            &CubeEyeCameraNode::getRunServiceCallback, this);
    mStopService = mNode.advertiseService("/cubeeye_camera_node/stop", 
                            &CubeEyeCameraNode::getStopServiceCallback, this);
    mDisconnectService = mNode.advertiseService("/cubeeye_camera_node/disconnect", 
                            &CubeEyeCameraNode::getDisconnectServiceCallback, this);
}

bool CubeEyeCameraNode::getLastStateServiceCallback(
    cubeeye_camera::LastState::Request& request,
    cubeeye_camera::LastState::Response& response)
{
    UNUSED(request);
    response.state = mCamera->getLastState();

    return true;
} 

bool CubeEyeCameraNode::getLastErrorServiceCallback(
    cubeeye_camera::LastError::Request& request,
    cubeeye_camera::LastError::Response& response)
{
    UNUSED(request);
    response.error = mCamera->getLastError();

    return true;
} 

bool CubeEyeCameraNode::getScanServiceCallback(cubeeye_camera::Scan::Request& request,
                                cubeeye_camera::Scan::Response& response)
{
    UNUSED(request);
    ROS_INFO("scan cameras...");
    response.connections = mCamera->scan();

    return true;
}

bool CubeEyeCameraNode::getConnectServiceCallback(cubeeye_camera::Connect::Request& request,
                                    cubeeye_camera::Connect::Response& response)
{
    ROS_INFO("connect camera(index: %d)", request.index);
    response.result = mCamera->connect(request.index);
    if (response.result != meere::sensor::result::success) {
        ROS_ERROR("camera connection failed.");
        return false;
    }

    mCamera->connectTo(mNode);

    return true;
}

bool CubeEyeCameraNode::getRunServiceCallback(cubeeye_camera::Run::Request& request,
                                cubeeye_camera::Run::Response& response)
{
    ROS_INFO("run camera(type: %d)", request.type);
    response.result = mCamera->run(request.type);

    return true;
}

bool CubeEyeCameraNode::getStopServiceCallback(cubeeye_camera::Stop::Request& request,
                                cubeeye_camera::Stop::Response& response)
{
    UNUSED(request);
    ROS_INFO("stop camera");    
    response.result = mCamera->stop();

    return true;
}

bool CubeEyeCameraNode::getDisconnectServiceCallback(cubeeye_camera::Disconnect::Request& request,
                                        cubeeye_camera::Disconnect::Response& response)
{
    UNUSED(request);
    ROS_INFO("disconnect camera");
    response.result = mCamera->disconnect();
    if (response.result != meere::sensor::result::success) {
        ROS_ERROR("camera disconnection failed.");
        return false;
    }

    mCamera->disconnectFrom(mNode);

    return true;
}

bool CubeEyeCameraNode::shutdown()
{
    ROS_INFO("node shutdown");
    if (mCamera != nullptr) {
        mCamera->shutdown();
    }
    return true;
}

void sigint_handler(int sigNum) {
    CubeEyeCameraNode::getInstance()->shutdown();
    ROS_INFO("cubeeye camera node stopped");

    ros::shutdown();    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cubeeye_camera_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    auto node = CubeEyeCameraNode::getInstance();
    node->init(nh);

    signal(SIGINT, sigint_handler);

    ROS_INFO("cubeeye camera node started");

    ros::spin();
    return 0;
}

