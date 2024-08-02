#ifndef CUBEEYE_CAMERA_NODE_H_
#define CUBEEYE_CAMERA_NODE_H_

class CameraModule;

class CubeEyeCameraNode
{
public:
    CubeEyeCameraNode();
    virtual ~CubeEyeCameraNode() = default;

    void init(ros::NodeHandle node);
    bool shutdown();

    static std::shared_ptr<CubeEyeCameraNode> getInstance();

protected:
    bool getLastStateServiceCallback(cubeeye_camera::LastState::Request& request, 
                                    cubeeye_camera::LastState::Response& response);
    bool getLastErrorServiceCallback(cubeeye_camera::LastError::Request& request, 
                                    cubeeye_camera::LastError::Response& response);
    bool getScanServiceCallback(cubeeye_camera::Scan::Request& request,
                                    cubeeye_camera::Scan::Response& response);
    bool getConnectServiceCallback(cubeeye_camera::Connect::Request& request,
                                    cubeeye_camera::Connect::Response& response);
    bool getRunServiceCallback(cubeeye_camera::Run::Request& request,
                                    cubeeye_camera::Run::Response& response);
    bool getStopServiceCallback(cubeeye_camera::Stop::Request& request,
                                    cubeeye_camera::Stop::Response& response);
    bool getDisconnectServiceCallback(cubeeye_camera::Disconnect::Request& request,
                                        cubeeye_camera::Disconnect::Response& response);
private:
    ros::ServiceServer mLastStateService;
    ros::ServiceServer mLastErrorService;
    ros::ServiceServer mScanService;
    ros::ServiceServer mConnectService;
    ros::ServiceServer mRunService;
    ros::ServiceServer mStopService;
    ros::ServiceServer mDisconnectService;

    ros::NodeHandle mNode;
    std::shared_ptr<CameraModule> mCamera;

    static std::shared_ptr<CubeEyeCameraNode> mInstance;
};


#endif // CubeEyeCameraNode
