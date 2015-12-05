//----------------------------------------------------------------------------------------
/**
* \file       camera_factory.h
* \author     Filip Srajer
* \date       December 2015
* \brief      Register of all derived cameras during compile time.
*
*  Classes for constructing cameras from files and for registering names of all 
*  derived cameras during compile time which is needed by the factory.
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <istream>
#include <memory>
#include <string>

#include "defines.h"
#include "camera.h"

using namespace yasfm;
using std::istream;
using std::make_unique;
using std::string;
using std::unique_ptr;

////////////////////////////////////////////////////
///////////////   Declarations   ///////////////////
////////////////////////////////////////////////////

namespace yasfm
{

/// Factory for creating cameras from a file.
/** 
Given a derived camera name, this creates a Camera pointer to the 
new instance of the derived class which gets constructed from a file.
This assumes that given derived camera registered its name using CameraRegister.
*/
class CameraFactory
{
public:
  /// Create a Camera pointer to a new derived camera.
  /**
  \param[in] className Name of the derived camera with which it was registered.
  \param[in,out] file Opened input file to construct camera.
  \param[in] camReadMode Camera::ReadMode to construct camera.
  \param[in] featuresDir Directory with features file to construct camera.
  \return Camera pointer to the new derived camera.
  */
  YASFM_API static unique_ptr<Camera> createInstance(const string& className,
    istream& file,int camReadMode,const string& featuresDir);

protected:
  /// Shortcut name.
  typedef umap<string,unique_ptr<Camera>(*)(istream&,int,
    const string&)> MapType;

  /// \return Reference to the register with cameras. A class name is the key 
  /// and the value is a function creating a derived camera.
  static MapType& map();

private:
  static MapType *map_; ///< Register with camera names and their constructors.
};

/// Register for all classes derived from Camera.
/**
This is used to determine which camera should be constructed based
on their names. That is useful for writing out cameras and then reading
and instantiating them.
T is the camera class being registered.
*/
template<class T>
class CameraRegister : private CameraFactory
{
public:
  /// Register a derived camera name.
  /// \param[in] className Exact class name (e.g. "DerivedCamera" for DerivedCamera).
  CameraRegister(const string& className);
};

} // namespace yasfm

namespace
{

/// Make a new derived camera. The new camera class is given by T.
/**
\param[in,out] file Opened input file to construct camera.
\param[in] camReadMode Camera::ReadMode to construct camera.
\param[in] featuresDir Directory with features file to construct camera.
\return Camera pointer to the new derived camera.
*/
template<class T>
unique_ptr<Camera> createCamera(istream& file,int camReadMode,
  const string& featuresDir);

} // namespace

////////////////////////////////////////////////////
///////////////   Definitions   ////////////////////
////////////////////////////////////////////////////

namespace yasfm
{

template<class T>
CameraRegister<T>::CameraRegister(const string& className)
{
  map()[className] = &createCamera<T>;
}

} // namespace yasfm


namespace
{

template<class T>
unique_ptr<Camera> createCamera(istream& file,int camReadMode,
  const string& featuresDir)
{
  return make_unique<T>(file,camReadMode,featuresDir);
}

} // namespace