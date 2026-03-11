/*
 * TI Voxel Lib component - Open3D Integration
 */

#include "Open3DGrabber.h"

namespace Voxel
{

Open3DGrabber::Open3DGrabber(DepthCamera &depthCamera):
  _depthCamera(depthCamera), _pointCloudSignal(nullptr)
{
  _depthCamera.registerCallback(DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME,
    [this](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType type) {
      this->_callback(dc, frame, type);
    });
}

Open3DGrabber::~Open3DGrabber()
{
  if (_pointCloudSignal)
  {
    delete _pointCloudSignal;
  }
}

float Open3DGrabber::getFramesPerSecond() const
{
  FrameRate r;
  if (!_depthCamera.getFrameRate(r))
    return 0;
  else
    return r.getFrameRate();
}

void Open3DGrabber::_callback(DepthCamera &depthCamera, const Frame &frame, DepthCamera::FrameType type)
{
  if (type != DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME || !_pointCloudSignal)
    return;

  const PointCloudFrame *pcFrame = dynamic_cast<const PointCloudFrame *>(&frame);
  if (!pcFrame)
    return;

  // Convert Voxel point cloud to Open3D point cloud
  auto o3d_cloud = std::make_shared<open3d::geometry::PointCloud>();
  
  size_t num_points = pcFrame->points.size();
  o3d_cloud->points_.reserve(num_points);
  o3d_cloud->colors_.reserve(num_points); // Open3D uses colors for intensities often

  for (size_t i = 0; i < num_points; ++i)
  {
    const IntensityPoint &pt = pcFrame->points[i];
    o3d_cloud->points_.emplace_back(pt.x, pt.y, pt.z);
    
    // Normalize intensity to pseudo-color (gray-scale)
    float i_norm = pt.i / 4095.0f; // Assuming 12-bit max intensity
    o3d_cloud->colors_.emplace_back(i_norm, i_norm, i_norm);
  }

  // Fire callback block with the open3d pointer
  if (_pointCloudSignal && !_pointCloudSignal->empty())
  {
    (*_pointCloudSignal)(o3d_cloud);
  }
}

}
