#include <sstream>
#include <thread>
#include <signal.h>

#include <ros/ros.h>

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeIntensityPointCloudFrame.h"

#include "ICubeModelParameter.h"

ICubeModelParameter::ICubeModelParameter(meere::sensor::sptr_camera camera) : ModelParameter(camera) {
    mCallbackType = boost::bind(&ICubeModelParameter::parametersCallback, this, _1, _2);
    mServer.setCallback(mCallbackType);
}

enum Config {
    amplitude_threshold_max          = 1,
    amplitude_threshold_min          = 2,
    auto_exposure                    = 3,
    depth_range_min                  = 4,
    depth_range_max                  = 5,
    flying_pixel_remove_filter       = 6,
    flying_pixel_remove_threshold    = 7,
    framerate                        = 8,
    integration_time                 = 9,
    median_filter                    = 10,
    noise_removal_threshold          = 11,
    outlier_remove_filter            = 12,
    phase_noise_filter               = 13,
    scattering_filter                = 14,
    scattering_filter_threshold      = 15,
};

void ICubeModelParameter::parametersCallback(cubeeye_camera_node::cubeeye_icubeConfig &config, uint32_t level)
{
    meere::sensor::sptr_property _prop = nullptr;
    switch (level)
    {
    case amplitude_threshold_max :
        _prop = meere::sensor::make_property_16u("amplitude_threshold_max", config.amplitude_threshold_max);
        break;
    case amplitude_threshold_min :
        _prop = meere::sensor::make_property_16u("amplitude_threshold_min", config.amplitude_threshold_min);
        break;
    case auto_exposure :
        _prop = meere::sensor::make_property_bool("auto_exposure", config.auto_exposure);
        break;
    case depth_range_min :
        _prop = meere::sensor::make_property_16u("depth_range_min", config.depth_range_min);
        break;
    case depth_range_max :
        _prop = meere::sensor::make_property_16u("depth_range_max", config.depth_range_max);
        break;
    case flying_pixel_remove_filter :
        _prop = meere::sensor::make_property_bool("flying_pixel_remove_filter", config.flying_pixel_remove_filter);
        break;
    case flying_pixel_remove_threshold :
        _prop = meere::sensor::make_property_16u("flying_pixel_remove_threshold", config.flying_pixel_remove_threshold);
        break;
    case framerate :
        _prop = meere::sensor::make_property_8u("framerate", config.framerate);
        break;
    case integration_time :
        _prop = meere::sensor::make_property_16u("integration_time", config.integration_time);
        break;
    case median_filter :
        _prop = meere::sensor::make_property_bool("median_filter", config.median_filter);
        break;
    case noise_removal_threshold :
        _prop = meere::sensor::make_property_16u("noise_removal_threshold", config.noise_removal_threshold);
        break;
    case outlier_remove_filter :
        _prop = meere::sensor::make_property_bool("outlier_remove_filter", config.outlier_remove_filter);
        break;
    case phase_noise_filter :
        _prop = meere::sensor::make_property_bool("phase_noise_filter", config.phase_noise_filter);
        break;
    case scattering_filter :
        _prop = meere::sensor::make_property_bool("scattering_filter", config.scattering_filter);
        break;                                
    case scattering_filter_threshold :
        _prop = meere::sensor::make_property_16u("scattering_filter_threshold", config.scattering_filter_threshold);
        break;
    }//switch

    if (nullptr != _prop)
    {
        // set property
        if (meere::sensor::result::success != mCamera->setProperty(_prop))
        {
            // error
            std::cout << "setProperty(" << _prop->key() << ") failed." << std::endl;
        }
    }
}
