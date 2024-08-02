/*
 * CubeEyeSink.h
 *
 *  Created on: 2020. 1. 6.
 *      Author: erato
 */

#ifndef CUBEEYESINK_H_
#define CUBEEYESINK_H_

#include "CubeEyeCamera.h"
#include "CubeEyeFrameList.h"

BEGIN_NAMESPACE

class _decl_dll CubeEyeSink
{
public:
	virtual std::string _decl_call name() const = 0;
	virtual void _decl_call onCubeEyeCameraState(const ptr_source source, CameraState state) = 0;
	virtual void _decl_call onCubeEyeCameraError(const ptr_source source, CameraError error) = 0;
	virtual void _decl_call onCubeEyeFrameList(const ptr_source source, const sptr_frame_list& frames) = 0;
	virtual void _decl_call onCubeEyeMessage(const ptr_source source, const std::string& msg, const std::string& contentType) {}

protected:
	CubeEyeSink() = default;
	virtual ~CubeEyeSink() = default;
};

using ptr_sink = CubeEyeSink*;
using sptr_sink = std::shared_ptr<CubeEyeSink>;

END_NAMESPACE

#endif /* CUBEEYESINK_H_ */
