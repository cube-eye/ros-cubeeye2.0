#ifndef ICUBE_MODEL_PARAMETER_H_
#define ICUBE_MODEL_PARAMETER_H_

#include "ModelParameter.h"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cubeeye_camera/cubeeye_icubeConfig.h>

class ICubeModelParameter : public ModelParameter
{
public:
    ICubeModelParameter(meere::sensor::sptr_camera camera);

    void parametersCallback(cubeeye_camera_node::cubeeye_icubeConfig &config, uint32_t level);

    dynamic_reconfigure::Server<cubeeye_camera_node::cubeeye_icubeConfig> mServer;
    dynamic_reconfigure::Server<cubeeye_camera_node::cubeeye_icubeConfig>::CallbackType mCallbackType;    
};

#endif // ICUBE_MODEL_PARAMETER_H_
