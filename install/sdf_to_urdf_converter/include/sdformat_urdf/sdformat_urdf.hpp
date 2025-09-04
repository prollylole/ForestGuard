#ifndef SDFORMAT_URDF_SDFORMAT_URDF_HPP
#define SDFORMAT_URDF_SDFORMAT_URDF_HPP

#include <memory>
#include <sdf/Error.hh>
#include <sdf/Root.hh>
#include <sdf/Model.hh>
#include <urdf_model/model.h>

// Forward declaration and type definition
namespace urdf
{
class ModelInterface;
typedef std::shared_ptr<ModelInterface> ModelInterfaceSharedPtr;
}

namespace sdformat_urdf
{

::urdf::ModelInterfaceSharedPtr parse(const std::string & data, sdf::Errors & errors);

::urdf::ModelInterfaceSharedPtr sdf_to_urdf(const sdf::Root & sdf_dom, sdf::Errors & errors);

::urdf::ModelInterfaceSharedPtr convert_model(const sdf::Model & sdf_model, sdf::Errors & errors);

}  // namespace sdformat_urdf

#endif  // SDFORMAT_URDF_SDFORMAT_URDF_HPP