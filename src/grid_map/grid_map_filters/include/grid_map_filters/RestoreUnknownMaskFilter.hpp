/*
 * RestoreUnknownMaskFilter.hpp
 *
 * Restores NaN cells in an output layer based on a mask layer.
 */

#pragma once

#include <filters/filter_base.h>
#include <grid_map_core/grid_map_core.hpp>

#include <string>

namespace grid_map {

class RestoreUnknownMaskFilter : public filters::FilterBase<GridMap> {
 public:
  RestoreUnknownMaskFilter();
  ~RestoreUnknownMaskFilter() override;

  bool configure() override;
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  std::string maskLayer_;
  std::string inputLayer_;
  std::string outputLayer_;
};

}  // namespace grid_map
