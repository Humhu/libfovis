#include "stereo_calibration.hpp"

#include <cstdio>
#include <iostream>

#include <emmintrin.h>

#include "stereo_rectify.hpp"

namespace fovis
{
StereoCalibration::StereoCalibration( const StereoCalibrationParameters& params )
	: _parameters( params )
{
	initialize();
}

StereoCalibration::StereoCalibration( const StereoCalibration& other )
{
  _parameters = other._parameters;
  _rectified_parameters = other._rectified_parameters;
  _left_rectification = std::make_shared<Rectification>( *other._left_rectification );
  _right_rectification = std::make_shared<Rectification>( *other._right_rectification );  
}

void
StereoCalibration::initialize()
{
	Eigen::Quaterniond rotation_quat( _parameters.right_to_left_rotation[0],
	                                  _parameters.right_to_left_rotation[1],
	                                  _parameters.right_to_left_rotation[2],
	                                  _parameters.right_to_left_rotation[3] );
	Eigen::Vector3d translation( _parameters.right_to_left_translation[0],
	                             _parameters.right_to_left_translation[1],
	                             _parameters.right_to_left_translation[2] );

	Eigen::Matrix3d left_rotation, right_rotation;
	stereo_rectify( _parameters.left_parameters, _parameters.right_parameters,
	                rotation_quat,
	                translation,
	                &left_rotation, &right_rotation, &_rectified_parameters );

	_left_rectification = std::make_shared<Rectification>( _parameters.left_parameters,
	                                                       left_rotation,
	                                                       _rectified_parameters );

	_right_rectification = std::make_shared<Rectification>( _parameters.right_parameters,
	                                                        right_rotation,
	                                                        _rectified_parameters );
}

} /*  */
