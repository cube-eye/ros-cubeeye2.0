#include <sstream>
#include <thread>
#include <signal.h>

#include <ros/ros.h>

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeIntensityPointCloudFrame.h"

#include "SCubeModelParameter.h"

SCubeModelParameter::SCubeModelParameter(meere::sensor::sptr_camera camera) : ModelParameter(camera) {
    mCallbackType = boost::bind(&SCubeModelParameter::parametersCallback, this, _1, _2);
    mServer.setCallback(mCallbackType);
}

enum Config{
    amplitude_threshold               = 1,
    scattering_threshold              = 2,
    auto_exposure                     = 3,
    integration_time                  = 4,
    standby_mode                      = 5,
    illumination                      = 6,
    flying_pixel_remove_filter        = 7,
    noise_filter1                     = 8,
    noise_filter2                     = 9,
    noise_filter3                     = 10,
    corner_mask                       = 11,
    depth_offset                      = 12,
    depth_range_min                   = 13,
    depth_range_max                   = 14,
    depth_error_remove_threshold      = 15,
};

void SCubeModelParameter::parametersCallback(cubeeye_camera_node::cubeeye_scubeConfig &config, uint32_t level)
{
    meere::sensor::sptr_property _prop = nullptr;
    switch (level)
    {
    case amplitude_threshold :
        _prop = meere::sensor::make_property_16u("amplitude_threshold_min", config.amplitude_threshold);
        break;
    case scattering_threshold :
        _prop = meere::sensor::make_property_16u("scattering_threshold", config.scattering_threshold);
        break;
    case auto_exposure :
        _prop = meere::sensor::make_property_bool("auto_exposure", config.auto_exposure);
        break;
    case integration_time :
        _prop = meere::sensor::make_property_8u("integration_time", config.integration_time);
        break;
    case standby_mode :
        _prop = meere::sensor::make_property_bool("standby_mode", config.standby_mode);
        break;
    case illumination :
        _prop = meere::sensor::make_property_bool("illumination", config.illumination);
        break;
    case flying_pixel_remove_filter :
        _prop = meere::sensor::make_property_bool("flying_pixel_remove_filter", config.flying_pixel_remove_filter);
        break;
    case noise_filter1 :
        _prop = meere::sensor::make_property_bool("noise_filter1", config.noise_filter1);
        break;
    case noise_filter2 :
        _prop = meere::sensor::make_property_bool("noise_filter2", config.noise_filter2);
        break;
    case noise_filter3 :
        _prop = meere::sensor::make_property_bool("noise_filter3", config.noise_filter3);
        break;
    case corner_mask :
        _prop = meere::sensor::make_property_bool("corner_mask", config.corner_mask);
        break;
    case depth_offset :
        _prop = meere::sensor::make_property_bool("depth_offset", config.depth_offset);
        break;
    case depth_range_min :
        _prop = meere::sensor::make_property_bool("depth_range_min", config.depth_range_min);
        break;
    case depth_range_max :
        _prop = meere::sensor::make_property_bool("depth_range_max", config.depth_range_max);
        break;
    case depth_error_remove_threshold :
        _prop = meere::sensor::make_property_8u("depth_error_remove_threshold", config.depth_error_remove_threshold);
        break;
    }//switch

    if (nullptr != _prop)
    {
        // set property
        if (meere::sensor::result::success != mCamera->setProperty(_prop))
        {
            // error
            ROS_ERROR("setProperty(%s) failed.", _prop->key().c_str());
        }
    }
}
