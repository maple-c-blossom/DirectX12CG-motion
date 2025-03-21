#pragma once
#include "IgnoreWarning.h"
WarningIgnoreBegin
#include <d3d12.h>
#include <vector>
WarningIgnoreEnd
#include "Descriptor.h"

namespace MCB
{
	class RootParameter
	{
		public:
			std::vector<D3D12_ROOT_PARAMETER> rootparams_;
		~RootParameter();

		void SetRootParam(const D3D12_ROOT_PARAMETER_TYPE& paramType,  uint32_t ShaderRegister,  uint32_t RegisterSpace,
			const D3D12_SHADER_VISIBILITY& shaderVisibility,  uint32_t NumDescriptorRanges,
			size_t descriptorIndex = 0);

		void SetRootParam(const D3D12_ROOT_PARAMETER_TYPE& paramType, 
			 uint32_t ShaderRegister = 0, size_t descriptorIndex = 0);

	};

}

