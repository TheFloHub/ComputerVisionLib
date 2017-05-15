/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  Match
/// 
///	@brief  
///

#pragma once

#include <Eigen/Core>
#include <limits>
#include <tuple>
#include <vector>

namespace Cvl
{

	class Match
	{

	public:

		Match() :
			mTemplateId(std::numeric_limits<size_t>::max()),
			mMeasuredId(std::numeric_limits<size_t>::max())
		{}

		Match(size_t templateId, size_t measuredId) :
			mTemplateId(templateId),
			mMeasuredId(measuredId)
		{}

		/// Creates two aligned arrays with source (first) and destination (second) points
		static std::tuple<Eigen::Array2Xd, Eigen::Array2Xd> alignMatches(
			Eigen::Array2Xd const& allSrcPoints,
			Eigen::Array2Xd const& allDstPoints,
			std::vector<Match> const& matches);
		
		size_t mTemplateId;

		size_t mMeasuredId;

	};
}





