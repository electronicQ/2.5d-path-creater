# Ubuntu 18.04's classic libtbb-dev has no TBBConfig.cmake.
# Require the classic API used by NormalVectorsFilter, rather than oneTBB.
find_path(TBB_INCLUDE_DIR NAMES tbb/task_scheduler_init.h)
find_library(TBB_LIBRARY NAMES tbb)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TBB DEFAULT_MSG TBB_INCLUDE_DIR TBB_LIBRARY)
set(TBB_INCLUDE_DIRS "${TBB_INCLUDE_DIR}")
set(TBB_LIBRARIES "${TBB_LIBRARY}")
mark_as_advanced(TBB_INCLUDE_DIR TBB_LIBRARY)
