#ifndef VEHICLE_SUPERVISOR__COMPONENT_SUPERVISOR_LAUNCH_HPP_
#define VEHICLE_SUPERVISOR__COMPONENT_SUPERVISOR_LAUNCH_HPP_

#include "component_supervisor.hpp"

class ASSupervisorLaunch : public ASSupervisor {
   private:    
    void run_fsm();
    void launch_mission();

   public:
    ASSupervisorLaunch(const rclcpp::NodeOptions & options);

};

#endif  // VEHICLE_SUPERVISOR__COMPONENT_SUPERVISOR_LAUNCH_HPP_