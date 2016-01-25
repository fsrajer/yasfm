#include "options_types.h"

namespace yasfm
{

OptTypeE OptTypeBool::type() const
{
  return OptTypeBoolE;
}

OptTypeE OptTypeInt::type() const
{
  return OptTypeIntE;
}

OptTypeE OptTypeFloat::type() const
{
  return OptTypeFloatE;
}

OptTypeE OptTypeDouble::type() const
{
  return OptTypeDoubleE;
}

OptTypeE OptTypeString::type() const
{
  return OptTypeStringE;
}

OptTypeE OptTypeFLANNIndex::type() const
{
  return OptTypeFLANNIndexE;
}

OptTypeE OptTypeFLANNSearch::type() const
{
  return OptTypeFLANNSearchE;
}

OptTypeE OptTypeCeres::type() const
{
  return OptTypeCeresE;
}

} // namespace yasfm