#pragma once

#include "PlayTypes.h"

namespace IDCT
{

	class CInterface
	{
	public:
		virtual			~CInterface() {} ;
		virtual void	Transform(const int16*, int16*) = 0;
	};

}
