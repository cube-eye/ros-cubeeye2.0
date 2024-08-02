#include <sstream>
#include <thread>
#include <signal.h>

#include <ros/ros.h>

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeIntensityPointCloudFrame.h"

#include "ModelParameter.h"
#include "ICubeModelParameter.h"
#include "SCubeModelParameter.h"

std::shared_ptr<ModelParameter> ModelParameter::create(meere::sensor::sptr_camera camera) {
    if (camera->source()->name() == "S100D"
        || camera->source()->name() == "S110D"
        || camera->source()->name() == "S111D"
        || camera->source()->name() == "E100") {
        return std::make_shared<SCubeModelParameter>(camera);
    }
    else if (camera->source()->name() == "I200D") {
        return std::make_shared<ICubeModelParameter>(camera);
    }

    return nullptr;
}
