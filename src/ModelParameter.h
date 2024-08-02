#ifndef MODEL_PARAMETER_H_
#define MODEL_PARAMETER_H_

class ModelParameter
{
public:
    struct Descriptor {
        std::string name;
        std::string description;
        uint8_t param_type;
        int data_type;
        int from_value;
        int to_value;
        int step;
        int default_value;
    };

    ModelParameter(meere::sensor::sptr_camera camera) : mCamera(camera) {};
    static std::shared_ptr<ModelParameter> create(meere::sensor::sptr_camera camera);

protected:
    meere::sensor::sptr_camera mCamera;
};

#endif // MODEL_PARAMETER_H_
