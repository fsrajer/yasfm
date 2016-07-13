#include "camera_factory.h"

namespace yasfm
{

CameraFactory::MapType* CameraFactory::map_;

unique_ptr<Camera> CameraFactory::createInstance(const string& className,
  istream& file)
{
  return (*map_)[className](file);
}

CameraFactory::MapType& CameraFactory::map()
{
  // No delete since it is active the whole time.
  if(!map_) map_ = new MapType;
  return (*map_);
}

} // namespace yasfm