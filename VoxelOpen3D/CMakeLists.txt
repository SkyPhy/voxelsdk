find_package(Voxel ${VOXEL_VERSION} REQUIRED)
find_package(Open3D REQUIRED)

add_library(voxelopen3d SHARED Open3DGrabber.cpp)
set_library_output_directory(voxelopen3d)
add_library(Voxel::voxelopen3d ALIAS voxelopen3d)

target_include_directories(voxelopen3d PRIVATE ${VOXEL_INCLUDE_DIRS} PUBLIC ${Open3D_INCLUDE_DIRS})
target_link_libraries(voxelopen3d LINK_PUBLIC ${VOXEL_LIBRARIES} LINK_PRIVATE Open3D::Open3D)

generate_export_header(voxelopen3d
  EXPORT_FILE_NAME "VoxelOpen3DExports.h"
)

install(TARGETS voxelopen3d
  EXPORT VoxelOpen3DTargets
  LIBRARY DESTINATION lib COMPONENT voxelopen3d
  ARCHIVE DESTINATION lib COMPONENT voxelopen3d
  RUNTIME DESTINATION lib COMPONENT voxelopen3d
)

install(FILES
  Open3DGrabber.h
  ${CMAKE_CURRENT_BINARY_DIR}/VoxelOpen3DExports.h
  DESTINATION include/voxel-${VOXEL_VERSION}/open3d
  COMPONENT voxelopen3d_dev
)

set_target_properties(voxelopen3d PROPERTIES 
                      VERSION ${VOXEL_VERSION} 
                      SOVERSION ${VOXEL_ABI_VERSION})

create_cmake_config(VoxelOpen3D "voxelopen3d" voxelopen3d_dev VoxelOpen3D)
