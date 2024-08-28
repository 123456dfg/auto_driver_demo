#ifndef _TRAFFIC_SIGN_H_
#define _TRAFFIC_SIGN_H_

#include <opencv2/core.hpp>
namespace driver_detector
{
	struct TrafficSign
	{
		int class_id;
		float confidence;
		cv::Rect box;
	};

	struct TransformStruct
	{
		TransformStruct() {}
		TransformStruct(float scale,
						float hh,
						float hw) : scale_(scale), hh_(hh), hw_(hw) {}
		float scale_;
		float hh_;
		float hw_;
	};
}

#endif // _TRAFFIC_SIGN_H_