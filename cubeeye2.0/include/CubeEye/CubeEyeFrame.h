/*
 * CubeEyeFrame.h
 *
 *  Created on: 2019. 12. 27.
 *      Author: erato
 */

#ifndef CUBEEYEFRAME_H_
#define CUBEEYEFRAME_H_

#include "CubeEyeData.h"
#include "CubeEyeProperty.h"

BEGIN_NAMESPACE

class _decl_dll CubeEyeFrame
{
public:
	enum FrameType {
		Unknown				= 0x000,
		Raw					= 0x001,
		Depth				= 0x002,
		Amplitude			= 0x004,
		Intensity			= 0x008,
		ZImage				= 0x010,
		PointCloud			= 0x020,
		ConfidenceMap		= 0x040,
		RGB					= 0x080,
		RegisteredDepth		= 0x100,
		RegisteredRGB		= 0x200,
		IntensityPointCloud	= 0x400,
		RegisteredPointCloud = 0x800,
		AdditionalInfo		= 0x8000,		
	};

public:
	virtual int32s _decl_call frameWidth() const = 0;
	virtual int32s _decl_call frameHeight() const = 0;
	virtual FrameType _decl_call frameType() const = 0;
	virtual DataType _decl_call frameDataType() const = 0;
	virtual std::string _decl_call frameFormat() const = 0;
	virtual int64u _decl_call timestamp() const = 0;

public:
	virtual result _decl_call setProperty(const sptr_property& property) = 0;
	virtual result_property _decl_call getProperty(const std::string& key) = 0;

protected:
	CubeEyeFrame() = default;
	virtual ~CubeEyeFrame() = default;
};


using FrameType = CubeEyeFrame::FrameType;
using sptr_frame = std::shared_ptr<CubeEyeFrame>;

END_NAMESPACE

#endif /* CUBEEYEFRAME_H_ */
