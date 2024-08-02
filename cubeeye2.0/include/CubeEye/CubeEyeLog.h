/*
 * CubeEyeLog.h
 *
 *  Created on: 2022. 10. 20.
 *      Author: erato
 */

#ifndef CUBEEYELOG_H_
#define CUBEEYELOG_H_

#include "CubeEye.h"

BEGIN_NAMESPACE

typedef enum CubeEyeLogLevel {
	Verbose,
	Debug,
	Info,
	Warn,
	Error
} CubeEyeLogLevel;

class _decl_dll CubeEyeLogListener
{
public:
	virtual void _decl_call onReceivedCubeEyeLog(CubeEyeLogLevel level, const std::string& msg) = 0;

protected:
	CubeEyeLogListener() = default;
	virtual ~CubeEyeLogListener() = default;
};


using log_level = CubeEyeLogLevel;
using log_listener = CubeEyeLogListener;
using ptr_log_listener = CubeEyeLogListener*;

_decl_dll result _decl_call add_log_listener(ptr_log_listener listener);
_decl_dll result _decl_call remove_log_listener(ptr_log_listener listener);
_decl_dll result _decl_call set_log(bool enable, log_level level = log_level::Warn, bool notify = false);

END_NAMESPACE

#endif /* CUBEEYELOG_H_ */
