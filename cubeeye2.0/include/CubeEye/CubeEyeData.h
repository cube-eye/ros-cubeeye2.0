/*
 * CubeEyeData.h
 *
 *  Created on: 2019. 12. 26.
 *      Author: erato
 */

#ifndef CUBEEYEDATA_H_
#define CUBEEYEDATA_H_

#include "CubeEye.h"

BEGIN_NAMESPACE

class _decl_dll CubeEyeData
{
public:
	enum DataType {
		Unknown,
		Boolean,
		S8,
		U8,
		S16,
		U16,
		S32,
		U32,
		F32,
		S64,
		U64,
		F64,
		Bytes,
		String
	};

public:
	virtual bool _decl_call isArray() const = 0;
	virtual bool _decl_call isNumeric() const = 0;
	virtual bool _decl_call isIntegral() const = 0;
	virtual bool _decl_call isString() const = 0;
	virtual DataType _decl_call dataType() const = 0;

protected:
	CubeEyeData() = default;
	virtual ~CubeEyeData() = default;
};


using DataType = CubeEyeData::DataType;

END_NAMESPACE

#endif /* CUBEEYEDATA_H_ */
