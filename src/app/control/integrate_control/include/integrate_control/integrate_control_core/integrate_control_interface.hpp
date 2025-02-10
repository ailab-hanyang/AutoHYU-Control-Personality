#ifndef _INTEGRATE_CONTROLLER_INTERFACE_HPP_
#define _INTEGRATE_CONTROLLER_INTERFACE_HPP_

#include <string>
#include <spline.h>
#include <boost/optional.hpp>

// Interface Header
#include "interface_constants.hpp"
#include "interface_trajectories.hpp"
#include "interface_vehicle_state.hpp"
#include "interface_control.hpp"


// using namespace tk;

namespace integrate_controller
{

class ControllerInterface
{
protected:

public:
  ControllerInterface();
  virtual ~ControllerInterface() = default;
  
  virtual void BuildControllerVehicleState(const interface::VehicleState& vehicle_state, tk::Map road_map) = 0;
  virtual boost::optional<interface::ControlCommand> CalculateOptimalCommand(const interface::ControlTrajectory& ref) = 0;
  
public:
  virtual std::string GetTargetFrame() = 0;
  virtual int GetControlTrajectoryStepNum() = 0;
  virtual double GetControlTrajectoryStepTime() = 0;
};
}  // integrate_controller
#endif  // _INTEGRATE_CONTROLLER_INTERFACE_HPP_
