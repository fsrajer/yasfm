//----------------------------------------------------------------------------------------
/**
* \file       options_types.h
* \brief      Types for options.
*
*  Types for storing various options in one array while being able to determine 
*  their types.
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <string>
#include <ostream>
#include <memory>

#include "ceres/solver.h" // Ceres options
#include "FLANN/util/params.h" // FLANN options

#include "defines.h"

using std::ostream;
using std::string;
using std::shared_ptr;

namespace yasfm
{

/// Enum for idetifying options types from the pointer to base class.
enum OptTypeE
{
  OptTypeBoolE,
  OptTypeIntE,
  OptTypeFloatE,
  OptTypeDoubleE,
  OptTypeStringE,
  OptTypeFLANNIndexE,
  OptTypeFLANNSearchE,
  OptTypeCeresE,
  OptTypeOptionsWrapperPtrE
};

/// Base class for options types.
class YASFM_API OptType
{
public:
  /// \return Identifier of the actual options type.
  virtual OptTypeE type() const = 0;
  virtual ~OptType() {}
};

typedef ptr_map<string,OptType> Options;

class OptionsWrapper
{
public:
  
  template<typename T>
  const T& get(const string& name) const
  {
    return static_cast<OptTypeWithVal<T> *>(&(*opt.at(name)))->val;
  }

  template<typename T>
  T& get(const string& name)
  {
    return static_cast<OptTypeWithVal<T> *>(&(*opt.at(name)))->val;
  }

  /// Write to a file.
  YASFM_API void write(ostream& file) const;

  Options opt;
};

typedef shared_ptr<OptionsWrapper> OptionsWrapperPtr;

template<typename T> 
class OptTypeWithVal : public OptType
{
public:
  YASFM_API OptTypeWithVal() {}
  YASFM_API OptTypeWithVal(T val) : val(val) {}
  YASFM_API virtual OptTypeE type() const;
  T val;
};


} // namespace yasfm