#ifndef SCUBE_MODEL_PARAMETER_H_
#define SCUBE_MODEL_PARAMETER_H_

#include "ModelParameter.h"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cubeeye_camera/cubeeye_scubeConfig.h>

class SCubeModelParameter : public ModelParameter
{
public:
    SCubeModelParameter(meere::sensor::sptr_camera camera);

    void parametersCallback(cubeeye_camera_node::cubeeye_scubeConfig &config, uint32_t level);

    dynamic_reconfigure::Server<cubeeye_camera_node::cubeeye_scubeConfig> mServer;
    dynamic_reconfigure::Server<cubeeye_camera_node::cubeeye_scubeConfig>::CallbackType mCallbackType;
};

#endif // SCUBE_MODEL_PARAMETER_H_
