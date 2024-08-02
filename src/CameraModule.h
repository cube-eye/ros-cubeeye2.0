#ifndef CAMERA_MODULE_H_
#define CAMERA_MODULE_H_

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeBasicFrame.h"
#include "CubeEyePointCloudFrame.h"
#include "CubeEyeIntensityPointCloudFrame.h"

#include "ModelParameter.h"

class ReceivedIntensityPCLFrameSink;

class CameraModule
{
public:
    CameraModule();

    // camera operations
    std::vector<std::string> scan();
    meere::sensor::result connect(int32_t index);
    meere::sensor::result run(int32_t type);
    meere::sensor::result stop();
    meere::sensor::result disconnect();
    void shutdown();

    // node connections
    void connectTo(ros::NodeHandle node);
    void disconnectFrom(ros::NodeHandle node);

    int32_t getLastState() { return mLastState.load(std::memory_order_acquire); }
    int32_t getLastError() { return mLastError.load(std::memory_order_acquire); };

private:
    void publishFrames(const meere::sensor::sptr_frame_list& frames);
    void createPublishers(ros::NodeHandle node);

    void setLastState(int32_t state) { mLastState.store(state, std::memory_order_release); }
    void setLastError(int32_t error) { mLastError.store(error, std::memory_order_release); }

    sensor_msgs::ImagePtr createImageMessage(meere::sensor::FrameType type, int32_t width, int32_t height);
    sensor_msgs::PointCloud2Ptr createPointCloudMessage(meere::sensor::FrameType type, int32_t width, int32_t height);

private:
    ros::Publisher mDepthImagePublisher;
    ros::Publisher mAmplitudeImagePublisher;
    ros::Publisher mRGBImagePublisher;
    ros::Publisher mPointCloudPublisher;

    std::shared_ptr<ReceivedIntensityPCLFrameSink> mSink;
    meere::sensor::sptr_source_list mSourceList;
    meere::sensor::sptr_camera mCamera;
    std::atomic<int32_t> mLastState;
    std::atomic<int32_t> mLastError;

    std::shared_ptr<ModelParameter> mModelParams;
    friend class ReceivedIntensityPCLFrameSink;
};

#endif // CAMERA_MODULE_H_
