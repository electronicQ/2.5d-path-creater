/*
 * RestoreUnknownMaskFilter.cpp
 *
 * Restores unknown cells from a source mask layer after intermediate filters
 * have used inpainted layers for slope/roughness/traversability calculations.
 */

#include "grid_map_filters/RestoreUnknownMaskFilter.hpp"

#include <cmath>

#include <ros/ros.h>

using namespace filters;

namespace grid_map {

RestoreUnknownMaskFilter::RestoreUnknownMaskFilter() = default;

RestoreUnknownMaskFilter::~RestoreUnknownMaskFilter() = default;

bool RestoreUnknownMaskFilter::configure() {
  if (!FilterBase::getParam(std::string("mask_layer"), maskLayer_)) {
    ROS_ERROR("RestoreUnknownMaskFilter did not find parameter 'mask_layer'.");
    return false;
  }
  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("RestoreUnknownMaskFilter did not find parameter 'input_layer'.");
    return false;
  }
  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("RestoreUnknownMaskFilter did not find parameter 'output_layer'.");
    return false;
  }
  return true;
}

bool RestoreUnknownMaskFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  if (!mapOut.exists(maskLayer_)) {
    ROS_ERROR_STREAM("RestoreUnknownMaskFilter missing mask layer: " << maskLayer_);
    return false;
  }
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR_STREAM("RestoreUnknownMaskFilter missing input layer: " << inputLayer_);
    return false;
  }

  Matrix output = mapOut[inputLayer_];
  const Matrix& mask = mapOut[maskLayer_];
  for (GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    if (!std::isfinite(mask(index(0), index(1)))) {
      output(index(0), index(1)) = NAN;
    }
  }
  mapOut.add(outputLayer_, output);
  return true;
}

}  // namespace grid_map
