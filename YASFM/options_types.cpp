#include "options_types.h"

namespace yasfm
{

// Explicit instantiation so that these would get exported.
template class OptTypeWithVal<bool>;
template class OptTypeWithVal<int>;
template class OptTypeWithVal<float>;
template class OptTypeWithVal<double>;
template class OptTypeWithVal<string>;
template class OptTypeWithVal<flann::IndexParams>;
template class OptTypeWithVal<flann::SearchParams>;
template class OptTypeWithVal<ceres::Solver::Options>;
template class OptTypeWithVal<OptionsWrapperPtr>;

OptionsWrapper::OptionsWrapper(const OptionsWrapper& o)
{
  opt.clear();
  for(const auto& entry : o.opt)
    opt.emplace(entry.first,entry.second->clone());
}

OptionsWrapper& OptionsWrapper::operator = (const OptionsWrapper& o)
{
  opt.clear();
  for(const auto& entry : o.opt)
    opt.emplace(entry.first,entry.second->clone());
  return *this;
}

void OptionsWrapper::write(ostream& file,const string& prefix) const
{
  for(const auto& entry : opt)
  {
    string field = entry.first;
    file << prefix << field << ": ";
    switch(entry.second->type())
    {
    case OptTypeBoolE:
      file << get<bool>(field) << "\n";
      break;
    case OptTypeIntE:
      file << get<int>(field) << "\n";
      break;
    case OptTypeFloatE:
      file << get<float>(field) << "\n";
      break;
    case OptTypeDoubleE:
      file << get<double>(field) << "\n";
      break;
    case OptTypeStringE:
      file << get<string>(field) << "\n";
      break;
    case OptTypeFLANNIndexE:
    {
      const auto& indexOpt = get<flann::IndexParams>(field);
      file << "\n";
      for(const auto& entry : indexOpt)
      {
        file << prefix << " " << entry.first << ": " << entry.second << "\n";
      }
      break;
    }
    case OptTypeFLANNSearchE:
    {
      const auto& searchOpt = get<flann::SearchParams>(field);
      file << "\n";
      file << prefix << " checks: " << searchOpt.checks << "\n";
      file << prefix << " eps: " << searchOpt.eps << "\n";
      file << prefix << " sorted: " << searchOpt.sorted << "\n";
      file << prefix << " max_neighbors: " << searchOpt.max_neighbors << "\n";
      file << prefix << " cores: " << searchOpt.cores << "\n";
      break;
    }
    case OptTypeCeresE:
    {
      const auto& ceresOpt = get<ceres::Solver::Options>(field);
      file << "\n";
      file << prefix << " max_num_iterations: " << ceresOpt.max_num_iterations << "\n";
      file << prefix << " num_threads: " << ceresOpt.num_threads << "\n";
      file << prefix << " function_tolerance: " << ceresOpt.function_tolerance << "\n";
      file << prefix << " parameter_tolerance: " << ceresOpt.parameter_tolerance << "\n";
      file << prefix << " gradient_tolerance: " << ceresOpt.gradient_tolerance << "\n";
      file << prefix << " minimizer_type: " << ceresOpt.minimizer_type << "\n";
      file << prefix << " linear_solver_type: " << ceresOpt.linear_solver_type << "\n";
      break;
    }
    case OptTypeOptionsWrapperPtrE:
      file << "\n";
      get<OptionsWrapperPtr>(field)->write(file," ");
      break;
    default:
      file << "unknown type";
      break;
    }
  }
}

template<>
OptTypeE OptTypeWithVal<bool>::type() const
{
  return OptTypeBoolE;
}

template<>
OptTypeE OptTypeWithVal<int>::type() const
{
  return OptTypeIntE;
}

template<>
OptTypeE OptTypeWithVal<float>::type() const
{
  return OptTypeFloatE;
}

template<>
OptTypeE OptTypeWithVal<double>::type() const
{
  return OptTypeDoubleE;
}

template<>
OptTypeE OptTypeWithVal<string>::type() const
{
  return OptTypeStringE;
}

template<>
OptTypeE OptTypeWithVal<flann::IndexParams>::type() const
{
  return OptTypeFLANNIndexE;
}

template<>
OptTypeE OptTypeWithVal<flann::SearchParams>::type() const
{
  return OptTypeFLANNSearchE;
}

template<>
OptTypeE OptTypeWithVal<ceres::Solver::Options>::type() const
{
  return OptTypeCeresE;
}

template<>
OptTypeE OptTypeWithVal<OptionsWrapperPtr>::type() const
{
  return OptTypeOptionsWrapperPtrE;
}

} // namespace yasfm