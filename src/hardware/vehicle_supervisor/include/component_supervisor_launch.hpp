#ifndef VEHICLE_SUPERVISOR__COMPONENT_SUPERVISOR_LAUNCH_HPP_
#define VEHICLE_SUPERVISOR__COMPONENT_SUPERVISOR_LAUNCH_HPP_

#include "component_supervisor.hpp"

namespace vehicle_supervisor {

class ASSupervisorLaunch : public ASSupervisor {
   private:
    void run_fsm();
    void launch_mission();

    void dvl_heartbeat_timer_callback();

   public:
    ASSupervisorLaunch(const rclcpp::NodeOptions& options);
};

}  // namespace vehicle_supervisor

#endif  // VEHICLE_SUPERVISOR__COMPONENT_SUPERVISOR_LAUNCH_HPP_
