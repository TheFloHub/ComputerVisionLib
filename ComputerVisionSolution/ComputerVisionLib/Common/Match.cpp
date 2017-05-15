#include "Match.h"

std::tuple<Eigen::Array2Xd, Eigen::Array2Xd> Cvl::Match::alignMatches(
	Eigen::Array2Xd const & allSrcPoints, 
	Eigen::Array2Xd const & allDstPoints, 
	std::vector<Match> const & matches)
{
	Eigen::Matrix2Xd src(2, matches.size());
	Eigen::Matrix2Xd dst(2, matches.size());
	int i = 0;
	for (auto const& match : matches)
	{
		src.col(i) = allSrcPoints.col(match.mTemplateId);
		dst.col(i) = allDstPoints.col(match.mMeasuredId);
		++i;
	}
	return std::make_tuple(src, dst);
}
