/*
 * CubeEyeAdditionalInfoFrame.h
 *
 *  Created on: 2023. 2. 9.
 *      Author: yjlim
 */

#ifndef CUBEEYEADDITIONALINFOFRAME_H_
#define CUBEEYEADDITIONALINFOFRAME_H_

#include "CubeEyeFrame.h"

BEGIN_NAMESPACE

template <typename T>
class _decl_dll CubeEyeAdditionalInfoFrame : public CubeEyeFrame
{
public:
	virtual CubeEyeList<T>* _decl_call frameDataRDC() const = 0;		// Raw Data Consistency 
	virtual CubeEyeList<T>* _decl_call frameDataNML() const = 0;	// Noise Model Level
	virtual CubeEyeList<T>* _decl_call frameDataIAC() const = 0;	// Intensity Amplitude Confidence
	virtual CubeEyeList<T>* _decl_call frameDataMFDE() const = 0;	// Multiple Frequency Depth Error
	virtual CubeEyeList<T>* _decl_call frameDataCFM() const = 0;		// Confidence Map

protected:
	CubeEyeAdditionalInfoFrame() = default;
	virtual ~CubeEyeAdditionalInfoFrame() = default;
};


using sptr_frame_ai16u = std::shared_ptr<CubeEyeAdditionalInfoFrame<int16u>>;
using sptr_frame_ai32f = std::shared_ptr<CubeEyeAdditionalInfoFrame<flt32>>;
using sptr_frame_ai64f = std::shared_ptr<CubeEyeAdditionalInfoFrame<flt64>>;

_decl_dll sptr_frame_ai16u _decl_call frame_cast_ai16u(const sptr_frame& frame);
_decl_dll sptr_frame_ai32f _decl_call frame_cast_ai32f(const sptr_frame& frame);
_decl_dll sptr_frame_ai64f _decl_call frame_cast_ai64f(const sptr_frame& frame);

END_NAMESPACE

#endif /* CUBEEYEPOINTCLOUDFRAME_H_ */
