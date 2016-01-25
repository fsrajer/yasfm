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

#include "ceres/solver.h" // Ceres options
#include "FLANN/util/params.h" // FLANN options

#include "defines.h"

using std::string;

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
  OptTypeCeresE
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
  Options opt;
};


class YASFM_API OptTypeBool : public OptType
{
public:
  virtual OptTypeE type() const;
  bool val;
};

class YASFM_API OptTypeInt : public OptType
{
public:
  virtual OptTypeE type() const;
  int val;
};

class YASFM_API OptTypeFloat : public OptType
{
public:
  virtual OptTypeE type() const;
  float val;
};

class YASFM_API OptTypeDouble : public OptType
{
public:
  virtual OptTypeE type() const;
  double val;
};

class YASFM_API OptTypeString : public OptType
{
public:
  virtual OptTypeE type() const;
  string val;
};

class YASFM_API OptTypeFLANNIndex : public OptType
{
public:
  virtual OptTypeE type() const;
  flann::IndexParams val;
};

class YASFM_API OptTypeFLANNSearch : public OptType
{
public:
  virtual OptTypeE type() const;
  flann::SearchParams val;
};

class YASFM_API OptTypeCeres : public OptType
{
public:
  virtual OptTypeE type() const;
  ceres::Solver::Options val;
};

} // namespace yasfm