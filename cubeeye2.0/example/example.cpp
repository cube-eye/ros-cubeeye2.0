/*
 * example.cpp
 *
 *  Created on: 2018. 10. 15.
 *      Author: erato
 */

#include <tuple>
#include <mutex>
#include <thread>
#include <queue>
#include <array>
#include <map>
#include <regex>
#include <vector>
#include <atomic>
#include <iostream>
#include <functional>
#include <condition_variable>

#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeBasicFrame.h"
#include "CubeEyePointCloudFrame.h"
#include "CubeEyeIntensityPointCloudFrame.h"

static const char*	CMD_SEARCH			= "search";
static const char*	CMD_SELECT			= "select";
static const char*	CMD_RUN				= "run";
static const char*	CMD_SETPROPERTY		= "set";
static const char*	CMD_GETPROPERTY		= "get";
static const char*	CMD_PROPERTY_VALUE	= "value";
static const char*	CMD_PROPERTY_LIST	= "property_list";
static const char*	CMD_FRAME_LIST		= "frame_list";
static const char*	CMD_HELP			= "help";

static const char*	OPT_INDEX			= "idx";
static const char*	OPT_TYPE			= "type";

static const int ITEM_PROPERTY			= 0;
static const int ITEM_PROPERTIES		= 1;
static const int ITEM_PROPERTIES2		= 2;
static const int DEFAULT_FRAME_TYPE		= meere::sensor::FrameType::Depth | meere::sensor::FrameType::Amplitude;

using cmd_arguments = std::tuple<meere::sensor::sptr_property, meere::sensor::sptr_properties>;

static std::map<std::string, std::string> _cmd_list = {
		{CMD_SEARCH, ""},
		{CMD_SELECT, ""},
		{CMD_RUN, ""},
		{CMD_SETPROPERTY, ""},
		{CMD_GETPROPERTY, ""},
		{CMD_PROPERTY_VALUE, ""},
};

static struct option _cmd[] = {
		{CMD_HELP, no_argument, 0, 'h'},
		{CMD_SEARCH, no_argument, 0, 0},
		{CMD_SELECT, required_argument, 0, 'n'},
		{CMD_RUN, required_argument, 0, 'r'},
		{CMD_SETPROPERTY, required_argument, 0, 's'},
		{CMD_GETPROPERTY, required_argument, 0, 'g'},
		{CMD_PROPERTY_VALUE, required_argument, 0, 'v'},
		{CMD_PROPERTY_LIST, no_argument, 0, 'p'},
		{CMD_FRAME_LIST, no_argument, 0, 'f'},
		{0, 0, 0, 0}
};

static std::map<std::string, int> FRAME_LIST = {
		{"depth", meere::sensor::FrameType::Depth},
		{"amplitude", meere::sensor::FrameType::Amplitude},
		{"intensity", meere::sensor::FrameType::Intensity},
		{"zimage", meere::sensor::FrameType::ZImage},
		{"point-cloud", meere::sensor::FrameType::PointCloud},
		{"confidence-map", meere::sensor::FrameType::ConfidenceMap},
		{"rgb", meere::sensor::FrameType::RGB}
};

static std::map<std::string, int> PROPERTY_LIST = {
		{"temperature", ITEM_PROPERTY},
		{"auto_exposure", ITEM_PROPERTY},
		{"illumination", ITEM_PROPERTY},
		{"amplitude_threshold_min", ITEM_PROPERTY},
		{"amplitude_threshold_max", ITEM_PROPERTY},
		{"depth_range_min", ITEM_PROPERTY},
		{"depth_range_max", ITEM_PROPERTY},
		{"depth_offset", ITEM_PROPERTY},
		{"noise_filter1", ITEM_PROPERTY},
		{"noise_filter2", ITEM_PROPERTY},
		{"noise_filter3", ITEM_PROPERTY},
		{"flying_pixel_remove_filter", ITEM_PROPERTY},
		{"framerate", ITEM_PROPERTY},
		{"reboot", ITEM_PROPERTY},
		{"rgbd_registration", ITEM_PROPERTY},
		{"device_configuration", ITEM_PROPERTIES},
		{"network_configuration", ITEM_PROPERTIES},
		{"user_message", ITEM_PROPERTIES2}
};

static std::thread _run_thread;
static std::atomic<bool> _start = false;

static bool isIntegral(const std::string& str)
{
    static const std::regex _check_regex{ R"(\\+|-?[[:digit:]]+)" };

    if (!str.empty()) {
    	return std::regex_match(str, _check_regex);
    }

    return false;
}

static bool isNumeric(const std::string& str)
{
    static const std::regex _check_regex{ R"([+\-]?(?:0|[1-9]\d*)(?:\.\d*)?(?:[eE][+\-]?\d+)?)" };

    if (!str.empty()) {
    	return std::regex_match(str, _check_regex);
    }

    return false;
}

static std::vector<std::string> split(const std::string& str, const std::string& delimiter = "=")
{
	std::vector<std::string> _ret;
    if(0 == delimiter.length() || std::string::npos == str.find(delimiter)) {
    	return _ret;
    }

    std::size_t _find_offset = 0, _split_offset = 0;
    while (std::string::npos != (_split_offset = str.find(delimiter, _find_offset))) {
    	_ret.push_back(str.substr(_find_offset, _split_offset - _find_offset));
    	_find_offset = _split_offset + delimiter.length();
    }
    _ret.push_back(str.substr(_find_offset, str.length() - _find_offset));
    return _ret;
}

static void show_usage()
{
    std::cout << "Usage: " << "example" << " [command] [option]...\n"
              << "commands:\n"
              << "\t--search\t\tSearching for camera.\n"
              << "\t-n,--select\t\tSelect the camera by serial number from the searched list.\n"
              << "\t-r,--run\t\tStarts capturing. with frame type(default depth|amplitude)\n"
              << "\t-s,--set\t\tSet the camera's properties. requires option : -v\n"
              << "\t-g,--get\t\tGet the camera's properties.\n"
              << "\t-p,--property_list\t\tShows a list of available properties.\n"
              << "\t-f,--frame_list\t\tShows a list of available frames.\n"
              << "options:\n"
              << "\t-h,--help\t\tShow this help message.\n"
              << "\t-n\t\t\tThe serial number of the camera you want to select.\n"
              << "\t-v\t\t\tSpecifies the value to use for the property.\n"
              << "e.g.:\n"
              << "\t--search\n"
              << "\t--select NNK9Y012099132 --run 6\n"
              << "\t--select NNK9Y012099132 --get amplitude_threshold_min\n"
              << "\t--select NNK9Y012099132 --set amplitude_threshold_min --value 25\n"
              << "\t-n NNK9Y012099132 -s network_configuration -v \"Type=DHCP\"\n"
              << "\t-n NNK9Y012099132 -s network_configuration -v \"Type=STATIC&IP=192.168.120.73&Subnet=255.255.255.0&Gateway=192.168.120.1\"\n"
              << std::endl;
}

static void show_frame_list()
{
	std::cout << "Frame Types: " << std::endl;

    for (const auto& it : FRAME_LIST) {
    	std::cout << "\t" << it.first.c_str() << "\t"
    			<< (9 > it.first.size() ? "\t\t" : "\t")
    			<< it.second << std::endl;
    }

    std::cout << "\r\nUsage: " << "-f [frame type] or [frame type] | [frame type]\n"
    		<< "\te.g. If you want depth and amplitude, use "
			<< (meere::sensor::FrameType::Depth | meere::sensor::FrameType::Amplitude)
			<< "(" << meere::sensor::FrameType::Depth << "|" << meere::sensor::FrameType::Amplitude << ")." << std::endl;
}

static void show_property_list()
{
	std::cout << "Properties: " << std::endl;

	for (const auto& it : PROPERTY_LIST) {
		std::cout << "\t" << it.first.c_str() << std::endl;
	}

    std::cout << "\r\nUsage: "
    		<< "\t-s or --set [name] -v [value]\n"
    		<< "\t-g or --get [name]\n"
    		<< "\te.g. set 'amplitude_threshold_min' to 25 : --set amplitude_threshold_min -v 25\n"
    		<< "\te.g. get 'framerate' : --get framerate\n"
    		<< "\te.g. set 'network_configuration' : --set network_configuration "
			<< "-v \"Type=STATIC&IP=192.168.0.10&Subnet=255.255.255.0&Gateway=192.168.0.1\"\n"
			<< std::endl;
}

static void showResult(meere::sensor::result rt, const std::string& prefix = "")
{
	std::cout << prefix << "-> result : ";

	switch (rt) {
	case meere::sensor::result::success:
		std::cout << "success" << std::endl;
		break;
	case meere::sensor::result::fail:
		std::cout << "fail" << std::endl;
		break;
	case meere::sensor::result::empty:
		std::cout << "empty" << std::endl;
		break;
	case meere::sensor::result::overflow:
		std::cout << "overflow" << std::endl;
		break;
	case meere::sensor::result::not_found:
		std::cout << "not found" << std::endl;
		break;
	case meere::sensor::result::not_exist:
		std::cout << "not exist" << std::endl;
		break;
	case meere::sensor::result::not_ready:
		std::cout << "not ready" << std::endl;
		break;
	case meere::sensor::result::not_supported:
		std::cout << "not supported" << std::endl;
		break;
	case meere::sensor::result::not_implemented:
		std::cout << "not implemented" << std::endl;
		break;
	case meere::sensor::result::not_initialized:
		std::cout << "not initialized" << std::endl;
		break;
	case meere::sensor::result::no_such_device:
		std::cout << "no such device" << std::endl;
		break;
	case meere::sensor::result::no_response:
		std::cout << "no response" << std::endl;
		break;
	case meere::sensor::result::invalid_parameter:
		std::cout << "invalid parameter" << std::endl;
		break;
	case meere::sensor::result::invalid_operation:
		std::cout << "invalid operation" << std::endl;
		break;
	case meere::sensor::result::invalid_data_type:
		std::cout << "invalid data type" << std::endl;
		break;
	case meere::sensor::result::out_of_memory:
		std::cout << "out of memory" << std::endl;
		break;
	case meere::sensor::result::out_of_resource:
		std::cout << "out of resource" << std::endl;
		break;
	case meere::sensor::result::out_of_range:
		std::cout << "out of range" << std::endl;
		break;
	case meere::sensor::result::already_exists:
		std::cout << "already exists" << std::endl;
		break;
	case meere::sensor::result::already_opened:
		std::cout << "already opened" << std::endl;
		break;
	case meere::sensor::result::already_running:
		std::cout << "already running" << std::endl;
		break;
	case meere::sensor::result::already_initialized:
		std::cout << "already initialized" << std::endl;
		break;
	case meere::sensor::result::using_resources:
		std::cout << "using resources" << std::endl;
		break;
	case meere::sensor::result::timeout:
		std::cout << "timeout" << std::endl;
		break;
	default:
		std::cout << "unknown" << std::endl;
		break;
	}
}

static void showProperty(const meere::sensor::sptr_property& prop, const std::string& prefix = "-> result : ")
{
	if (nullptr != prop && !prop->key().empty()) {
		std::cout << prefix << prop->key();
		switch (prop->dataType()) {
		case meere::sensor::DataType::Boolean:
			printf(", %d\n", prop->asBoolean());
			break;
		case meere::sensor::DataType::S8:
			printf(", %hhd\n", prop->asInt8s());
			break;
		case meere::sensor::DataType::U8:
			printf(", %hhu\n", prop->asInt8u());
			break;
		case meere::sensor::DataType::S16:
			printf(", %hd\n", prop->asInt16s());
			break;
		case meere::sensor::DataType::U16:
			printf(", %hu\n", prop->asInt16u());
			break;
		case meere::sensor::DataType::F32:
			printf(", %f\n", prop->asFlt32());
			break;
		case meere::sensor::DataType::F64:
			printf(", %f\n", prop->asFlt64());
			break;
		case meere::sensor::DataType::S32:
			printf(", %d\n", prop->asInt32s());
			break;
		case meere::sensor::DataType::U32:
			printf(", %u\n", prop->asInt32u());
			break;
		case meere::sensor::DataType::S64:
			printf(", %lld\n", (long long) prop->asInt64s());
			break;
		case meere::sensor::DataType::U64:
			printf(", %llu\n", (unsigned long long) prop->asInt64u());
			break;
		case meere::sensor::DataType::String:
			std::cout << ", " << prop->asString() << std::endl;
			break;
		default:
			std::cout << std::endl;
			break;
		}
	}
}

static void showProperties(const meere::sensor::result_properties& rtproperties)
{
	auto _rt = std::get<0>(rtproperties);
	if (meere::sensor::result::success == _rt) {
		auto _name = std::get<1>(rtproperties)->name();
		auto _list = std::get<1>(rtproperties)->list();

		std::cout << "-> properties(" << _name << ") : " << std::endl;
		for (const auto& it : (*_list)) {
			showProperty(it, std::string("\t+property : "));
		}
	}
	else {
		showResult(_rt);
	}
}

static void sigactHandler(int sig)
{
	_start = false;
}

static class ReceivedFrameSink : public meere::sensor::sink
 , public meere::sensor::prepared_listener
{
public:
	virtual std::string name() const {
		return std::string("ReceivedFrameSink");
	}

	virtual void onCubeEyeCameraState(const meere::sensor::ptr_source source, meere::sensor::CameraState state) {
		printf("%s:%d source(%s) state = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), static_cast<int>(state));
		
		if (meere::sensor::CameraState::Running == state) {
			mReadFrameThreadStart = true;
			mReadFrameThread = std::thread(ReceivedFrameSink::ReadFrameProc, this);
		}
		else if (meere::sensor::CameraState::Stopped == state) {
			mReadFrameThreadStart = false;
			if (mReadFrameThread.joinable()) {
				mReadFrameThread.join();
			}
		}
	}

	virtual void onCubeEyeCameraError(const meere::sensor::ptr_source source, meere::sensor::CameraError error) {
		printf("%s:%d source(%s) error = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), static_cast<int>(error));
	}

	virtual void onCubeEyeFrameList(const meere::sensor::ptr_source source , const meere::sensor::sptr_frame_list& frames) {
		if (mReadFrameThreadStart) {
			static constexpr size_t _MAX_FRAMELIST_SIZE = 4;
			if (_MAX_FRAMELIST_SIZE > mFrameListQueue.size()) {
				auto _copied_frame_list = meere::sensor::copy_frame_list(frames);
				if (_copied_frame_list) {
					mFrameListQueue.push(std::move(_copied_frame_list));
				}
			}
		}
	}

public:
	virtual void onCubeEyeCameraPrepared(const meere::sensor::ptr_camera camera) {
		printf("%s:%d source(%s)\n", __FUNCTION__, __LINE__, camera->source()->uri().c_str());
	}

protected:
	static void ReadFrameProc(ReceivedFrameSink* thiz) {

		auto frameName = [](const meere::sensor::FrameType& type) {
			std::string _name("Unknown");

			switch (type) {
			case meere::sensor::FrameType::Depth:
				_name = "Depth";
				break;
			case meere::sensor::FrameType::Amplitude:
				_name = "Amplitude";
				break;
			case meere::sensor::FrameType::Intensity:
				_name = "Intensity";
				break;
			case meere::sensor::FrameType::ZImage:
				_name = "ZImage";
				break;
			case meere::sensor::FrameType::PointCloud:
				_name = "PCL";
				break;
			case meere::sensor::FrameType::RGB:
				_name = "RGB";
				break;
			case meere::sensor::FrameType::IntensityPointCloud:
				_name = "IPCL";
				break;
			default:
				break;
			}

			return _name;
		};

		while (thiz->mReadFrameThreadStart) {
			if (thiz->mFrameListQueue.empty()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}

			auto _frames = std::move(thiz->mFrameListQueue.front());
			thiz->mFrameListQueue.pop();

			if (_frames) {
				static int _frame_cnt = 0;
				if (30 > ++_frame_cnt) {
					continue;
				}
				_frame_cnt = 0;

				for (const auto& it : (*_frames)) {
					printf("* GET frame : %d, "
							"frameWidth = %d "
							"frameHeight = %d "
							"frameDataType = %d "
							"timestamp = %lu \n",
							it->frameType(),
							it->frameWidth(),
							it->frameHeight(),
							it->frameDataType(),
							it->timestamp());

					int _frame_index = 0;
					auto _center_x = it->frameWidth() / 2;
					auto _center_y = it->frameHeight() / 2;
					auto _frame_name = frameName(it->frameType());

					switch (it->frameType()) {
					case meere::sensor::FrameType::Depth:
					case meere::sensor::FrameType::Amplitude:
					case meere::sensor::FrameType::Intensity:
					case meere::sensor::FrameType::ZImage:
					case meere::sensor::FrameType::ConfidenceMap:
						// 16bits data type
						if (it->frameDataType() == meere::sensor::DataType::U16) {
							// casting 16bits basic frame
							auto _sptr_basic_frame = meere::sensor::frame_cast_basic16u(it);
							auto _sptr_frame_data = _sptr_basic_frame->frameData();	// data array

							for (int y = 0 ; y < _sptr_basic_frame->frameHeight(); y++) {
								for (int x = 0 ; x < _sptr_basic_frame->frameWidth(); x++) {
									_frame_index = y * _sptr_basic_frame->frameWidth() + x;
									if (_center_x == x && _center_y == y) {
										printf("\t%s frame : position(%d,%d) data -> %d\n",
												_frame_name.c_str(), _center_x, _center_y, (*_sptr_frame_data)[_frame_index]);
									}
								}
							}
						}
						// 32bits floating-point
						else if (it->frameDataType() == meere::sensor::DataType::F32) {
							// casting 32bits basic frame
							auto _sptr_basic_frame = meere::sensor::frame_cast_basic32f(it);
							auto _sptr_frame_data = _sptr_basic_frame->frameData();	// data array

							for (int y = 0 ; y < _sptr_basic_frame->frameHeight(); y++) {
								for (int x = 0 ; x < _sptr_basic_frame->frameWidth(); x++) {
									_frame_index = y * _sptr_basic_frame->frameWidth() + x;
									if (_center_x == x && _center_y == y) {
										printf("\t%s frame : position(%d,%d) data -> %f\n",
												_frame_name.c_str(), _center_x, _center_y, (*_sptr_frame_data)[_frame_index] * 1000);
									}
								}
							}
						}
						break;
					case meere::sensor::FrameType::RGB:
						// 8bits data type
						if (it->frameDataType() == meere::sensor::DataType::U8) {
							// casting 8bits basic frame
							auto _sptr_basic_frame = meere::sensor::frame_cast_basic8u(it);
							auto _sptr_frame_data = _sptr_basic_frame->frameData();	// data array
							const meere::sensor::int8u* _ptr_data = _sptr_frame_data->data();

							if ("RGB888" == it->frameFormat()) {
								// TODO :
								/*
								 * i.e. convert to opencv::Mat
								 * cv::Mat(it->frameHeight(), it->frameWidth(), CV_8UC3, _ptr_data));
								 */
							}
							else if ("H264" == it->frameFormat() || "H265" == it->frameFormat()) {
								// TODO :
							}
						}
						break;
					case meere::sensor::FrameType::PointCloud:
						// 32bits floating-point
						if (it->frameDataType() == meere::sensor::DataType::F32) {
							// casting 32bits point cloud frame
							auto _sptr_pointcloud_frame = meere::sensor::frame_cast_pcl32f(it);
							auto _sptr_frame_dataX = _sptr_pointcloud_frame->frameDataX(); // x-point data array
							auto _sptr_frame_dataY = _sptr_pointcloud_frame->frameDataY(); // y-point data array
							auto _sptr_frame_dataZ = _sptr_pointcloud_frame->frameDataZ(); // z-point data array

							for (int y = 0 ; y < _sptr_pointcloud_frame->frameHeight(); y++) {
								for (int x = 0 ; x < _sptr_pointcloud_frame->frameWidth(); x++) {
									_frame_index = y * _sptr_pointcloud_frame->frameWidth() + x;
									if (_center_x == x && _center_y == y) {
										printf("\tPCL frame : position(%d,%d) data -> X(%f), Y(%f), Z(%f)\n", _center_x, _center_y, \
										(*_sptr_frame_dataX)[_frame_index] * 1000, (*_sptr_frame_dataY)[_frame_index] * 1000, (*_sptr_frame_dataZ)[_frame_index] * 1000);
									}
								}
							}
						}
						break;
					case meere::sensor::FrameType::IntensityPointCloud:
						// 32bits floating-point
						if (it->frameDataType() == meere::sensor::DataType::F32) {
							// casting 32bits point cloud frame
							auto _sptr_ipointcloud_frame= meere::sensor::frame_cast_ipcl32f(it);
							auto _sptr_frame_dataI = _sptr_ipointcloud_frame->frameDataI(); // i-point data array
							auto _sptr_frame_dataX = _sptr_ipointcloud_frame->frameDataX(); // x-point data array
							auto _sptr_frame_dataY = _sptr_ipointcloud_frame->frameDataY(); // y-point data array
							auto _sptr_frame_dataZ = _sptr_ipointcloud_frame->frameDataZ(); // z-point data array

							for (int y = 0 ; y < _sptr_ipointcloud_frame->frameHeight(); y++) {
								for (int x = 0 ; x < _sptr_ipointcloud_frame->frameWidth(); x++) {
									_frame_index = y * _sptr_ipointcloud_frame->frameWidth() + x;
									if (_center_x == x && _center_y == y) {
										printf("\tIPCL frame : position(%d,%d) data -> I(%f), X(%f), Y(%f), Z(%f)\n",
												_center_x, _center_y, \
												(*_sptr_frame_dataI)[_frame_index] * 1000, (*_sptr_frame_dataX)[_frame_index] * 1000, \
												(*_sptr_frame_dataY)[_frame_index] * 1000, (*_sptr_frame_dataZ)[_frame_index] * 1000);
									}
								}
							}
						}
						break;
					default:
						break;
					}
				}

			}
		}
	}

public:
	ReceivedFrameSink() = default;
	virtual ~ReceivedFrameSink() = default;

protected:
	bool mReadFrameThreadStart;
	std::thread mReadFrameThread;
	std::queue<meere::sensor::sptr_frame_list> mFrameListQueue;
} _ReceivedFrameSink;


// for 'rgbd_registration'
//
// 1. search & select device
// 2. set rgbd_registration 1
// 3. run 130 <-- must(rgb+depth)!
//
// You cannot change the 'rgbd_registration' mode in running state. Set it in stop or prepare state.

int main(int argc, char* argv[])
{
	struct sigaction _sigact;
	memset(&_sigact, '\0', sizeof(_sigact));
	_sigact.sa_handler = &sigactHandler;

	if (0 > sigaction(SIGTERM, &_sigact, NULL)) {
		printf("sigaction(SIGTERM) failed.\n");
		return -1;
	}

	if (0 > sigaction(SIGINT, &_sigact, NULL)) {
		printf("sigaction(SIGINT) failed.\n");
		return -1;
	}

	if (0 > sigaction(SIGTSTP, &_sigact, NULL)) {
		printf("sigaction(SIGTSTP) failed.\n");
		return -1;
	}

	auto _search_camera = [] (bool show = false) {
		auto _source_list = meere::sensor::search_camera_source();
		if (show && nullptr != _source_list) {
			int i = 0;
			for (const auto& it : (*_source_list)) {
				std::cout << i++ << ") source name : " << it->name() << \
						", serialNumber : " << it->serialNumber() << \
						", uri : " << it->uri() << std::endl;
			}
		}

		return _source_list;
	};

	auto _make_property = [] (const std::string& key, const std::string& value) {
		meere::sensor::sptr_property _rt = nullptr;
		if (!key.empty() && !value.empty()) {
			if (isNumeric(value)) {
				if (isIntegral(value)) {
					return (_rt = meere::sensor::make_property_32s(key, std::stoi(value)));
				}

				return (_rt = meere::sensor::make_property_32f(key, std::stof(value)));
			}

			_rt = meere::sensor::make_property_string(key, value);
		}

		return _rt;
	};

	auto _make_properties = [_make_property] (const std::string& name, const std::string& items) {
		meere::sensor::sptr_properties _rt = nullptr;
		if (!name.empty() && !items.empty()) {
			_rt = meere::sensor::make_properties(name);
			if (_rt) {
				auto _item_list = split(items, "&");
				if (_item_list.empty()) {
					_item_list.push_back(items);
				}

				for (const auto& it : _item_list) {
					auto _item = split(it, "=");
					if (2 == _item.size()) {
						auto _property = _make_property(_item[0], _item[1]);
						if (_property) {
							_rt->add(_property);
						}
					}
				}
			}
		}

		return _rt;
	};

	while (1) {
		int _idx = 0;
		auto _c = getopt_long(argc, argv, "r:s:g:v:n:fph?", _cmd, &_idx);
		if (-1 == _c) {
			break;
		}

		switch (_c) {
		case 0:
			if (0 != _cmd[_idx].flag) {
				break;
			}

			if (!strcmp(CMD_SEARCH, _cmd[_idx].name)) {
				_search_camera(true);
				return 0;
			}
			break;
		case 'r':
			if (!isIntegral(optarg)) {
				std::cout << "\t'--run' requires an integral argument" << std::endl;
				return -1;
			}
			_cmd_list[CMD_RUN] = std::string(optarg);
			break;
		case 's':
		case 'g':
			if (isNumeric(optarg)) {
				std::cout << "\t'--run' requires an string argument" << std::endl;
				return -1;
			}

			_cmd_list['s' == _c ? CMD_SETPROPERTY : CMD_GETPROPERTY] = std::string(optarg);
			break;
		case 'n':
		case 'v':
			_cmd_list['n' == _c ? CMD_SELECT : CMD_PROPERTY_VALUE] = std::string(optarg);
			break;
		case 'f':
			show_frame_list();
			return 0;
		case 'p':
			show_property_list();
			return 0;
		default:
			show_usage();
			return 0;
		}
	}

	meere::sensor::sptr_source_list _source_list = nullptr;
	meere::sensor::sptr_camera _camera = nullptr;
	meere::sensor::add_prepared_listener(&_ReceivedFrameSink);

	// searching...
	auto _it = _cmd_list.find(CMD_SEARCH);
	if (_cmd_list.end() != _it) {
		_source_list = _search_camera();
	}

	if (!_source_list || 0 == _source_list->size()) {
		std::cout << "Camera not found!" << std::endl;
		return -1;
	}

	// create a camera
	_it = _cmd_list.find(CMD_SELECT);
	if (_cmd_list.end() == _it || _it->second.empty()) {
		std::cout << "The serial number required to select a camera is missing." << std::endl;
		return -1;
	}

	for (const auto& it : (*_source_list)) {
		if (it->serialNumber() == _it->second) {
			_camera = meere::sensor::create_camera(it);
			break;
		}
	}

	if (!_camera) {
		std::cout << "No such device" << "(" << _it->second << ")" << std::endl;
		return -1;
	}
	else {
		_camera->addSink(&_ReceivedFrameSink);

		// prepare
		auto _rt = _camera->prepare();
		if (meere::sensor::success != _rt) {
			showResult(_rt, "prepare ");
			meere::sensor::destroy_camera(_camera);
			return -1;
		}

		// get lens parameters
		auto _lenses = _camera->lenses();
		std::cout << "count of Lenses : " << _lenses << std::endl;
		std::vector<meere::sensor::int8u> _idx;
		for (size_t i = 0; i < _lenses; i++) {
			std::cout << "Lens index : " << i << std::endl;
			_idx.push_back(i);

			auto _fov = _camera->fov(i);
			std::cout << "	FoV : " << std::get<0>(_fov) << "(H) x " << std::get<1>(_fov) << "(V)" << std::endl;

			meere::sensor::IntrinsicParameters parameters;
			if (meere::sensor::success == (_rt = _camera->intrinsicParameters(parameters, i))) {
				std::cout << "	IntrinsicParameters :" << std::endl;
				std::cout << "		FocalLength(fx) = " << parameters.focal.fx << std::endl;
				std::cout << "		FocalLength(fy) = " << parameters.focal.fy << std::endl;
				std::cout << "		PrincipalPoint(cx) = " << parameters.principal.cx << std::endl;
				std::cout << "		PrincipalPoint(cy) = " << parameters.principal.cy << std::endl;
			}

			meere::sensor::DistortionCoefficients coefficients;
			if (meere::sensor::success == (_rt = _camera->distortionCoefficients(coefficients, i))) {
				std::cout << "	DistortionCoefficients :" << std::endl;
				std::cout << "		RadialCoefficient(K1) = " << coefficients.radial.k1 << std::endl;
				std::cout << "		RadialCoefficient(K2) = " << coefficients.radial.k2 << std::endl;
				std::cout << "		RadialCoefficient(K3) = " << coefficients.radial.k3 << std::endl;
				std::cout << "		TangentialCoefficient(P1) = " << coefficients.tangential.p1 << std::endl;
				std::cout << "		TangentialCoefficient(P2) = " << coefficients.tangential.p2 << std::endl;
				std::cout << "		skewCoefficient = " << coefficients.skewCoefficient << std::endl;
			}
		}

		if (1 < _idx.size()) {
			meere::sensor::ExtrinsicParameters parameters;
			if (meere::sensor::result::success == (_rt = _camera->extrinsicParameters(parameters, _idx[0], _idx[1]))) {
				std::cout << "	ExtrinsicParameters :" << std::endl;
				for (int i = 0; i < 3; i++) {
					std::cout << "		RotationParameters(r1[" << i << "]) = " << parameters.rotation.r1[i] << std::endl;
					std::cout << "		RotationParameters(r2[" << i << "]) = " << parameters.rotation.r2[i] << std::endl;
					std::cout << "		RotationParameters(r3[" << i << "]) = " << parameters.rotation.r3[i] << std::endl;
				}
				std::cout << "		translation(tx) = " << parameters.translation.tx << std::endl;
				std::cout << "		translation(ty) = " << parameters.translation.ty << std::endl;
				std::cout << "		translation(tz) = " << parameters.translation.tz << std::endl;
			}
		}
	}

	// run
	_it = _cmd_list.find(CMD_RUN);
	if (_cmd_list.end() != _it && !_it->second.empty()) {
		auto _wanted_frame = std::stoi(_it->second);
		auto _rt = _camera->run(_wanted_frame);
		if (meere::sensor::success != _rt) {
			showResult(_rt, "run ");
			meere::sensor::destroy_camera(_camera);
			return -1;
		}

		_start = true;
		_run_thread = std::thread([] () {
			while (_start) {
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}
		});
	}

	// set property
	_it = _cmd_list.find(CMD_SETPROPERTY);
	if (_cmd_list.end() != _it && !_it->second.empty()) {
		auto _item = PROPERTY_LIST.find(_it->second);
		if (PROPERTY_LIST.end() == _item) {
			std::cout << "No such property" << "(" << _it->second << ")" << std::endl;
			return -1;
		}

		auto _value = _cmd_list.find(CMD_PROPERTY_VALUE);
		if (_cmd_list.end() == _value) {
			std::cout << "not found values" << std::endl;
			return -1;
		}

		if (ITEM_PROPERTY == _item->second) {
			auto _prop = _make_property(_item->first, _value->second);
			auto _rt = _camera->setProperty(_prop);
			showResult(_rt, "setProperty(" + _item->first + ")");
		}
		else if (ITEM_PROPERTIES == _item->second) {
			auto _prop = _make_properties(_item->first, _value->second);
			auto _rt = _camera->setProperties(_prop);
			showResult(_rt, "setProperties(" + _item->first + ")");
		}
		else if (ITEM_PROPERTIES2 == _item->second) {
			auto _prop = _make_properties(_item->first, _value->second);
			auto _rt = meere::sensor::set_properties(_prop);
			showResult(_rt, "set_properties(" + _item->first + ")");
		}
	}

	// get property
	_it = _cmd_list.find(CMD_GETPROPERTY);
	if (_cmd_list.end() != _it && !_it->second.empty()) {
		auto _item = PROPERTY_LIST.find(_it->second);
		if (PROPERTY_LIST.end() == _item) {
			std::cout << "No such property" << "(" << _it->second << ")" << std::endl;
			return -1;
		}

		if (ITEM_PROPERTY == _item->second) {
			auto _rt = _camera->getProperty(_item->first);
			if (meere::sensor::success == std::get<0>(_rt)) {
				showProperty(std::get<1>(_rt));
			}
			else {
				showResult(std::get<0>(_rt), "getProperty(" + _item->first + ")");
			}
		}
		else if (ITEM_PROPERTIES == _item->second) {
			auto _rt = _camera->getProperties(_item->first);
			showProperties(_rt);
		}
	}

	if (_run_thread.joinable()) {
		_run_thread.join();
	}

	meere::sensor::destroy_camera(_camera);
	_camera.reset();
	return 0;
}
