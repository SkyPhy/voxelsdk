/*
 * TI Voxel Lib component - Open3D Integration
 */

#ifndef VOXEL_OPEN3D_GRABBER_H
#define VOXEL_OPEN3D_GRABBER_H

#include <DepthCamera.h>
#include <open3d/Open3D.h>
#include <boost/signals2.h>

#include "VoxelOpen3DExports.h"

namespace Voxel
{

class VOXELOPEN3D_EXPORT Open3DGrabber
{
public:
  typedef void (PointCloudCallBack) (std::shared_ptr<open3d::geometry::PointCloud>);
  
protected:
  DepthCamera &_depthCamera;
  boost::signals2::signal<PointCloudCallBack>* _pointCloudSignal;
  
  void _callback(DepthCamera &depthCamera, const Frame &frame, DepthCamera::FrameType type);
  
public:
  Open3DGrabber(DepthCamera &depthCamera);
  virtual ~Open3DGrabber();
  
  virtual float getFramesPerSecond() const;
  virtual std::string getName() const { return _depthCamera.id(); }
  virtual bool isRunning() const { return _depthCamera.isRunning(); }
  
  virtual void start() { _depthCamera.start(); }
  virtual void stop() { _depthCamera.stop(); _depthCamera.wait(); }

  template<typename T> boost::signals2::connection registerCallback(const boost::function<T>& callback);
};

template<typename T>
boost::signals2::connection Open3DGrabber::registerCallback(const boost::function<T>& callback)
{
  if (typeid(T) == typeid(PointCloudCallBack))
  {
    if (!_pointCloudSignal)
      _pointCloudSignal = new boost::signals2::signal<PointCloudCallBack>();
    return _pointCloudSignal->connect(callback);
  }
  return boost::signals2::connection();
}

}

#endif // VOXEL_OPEN3D_GRABBER_H
