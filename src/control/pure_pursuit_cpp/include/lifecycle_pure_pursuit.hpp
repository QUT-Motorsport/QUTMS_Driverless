#pragma once

#include "pure_pursuit.hpp"

class PurePursuitLifecycle : public rclcpp_lifecycle::LifecycleNode {
   private:
    std::unique_ptr<PurePursuit> pp;

   public:
    PurePursuitLifecycle();
    void initialise();

    CallbackReturn on_configure(rclcpp_lifecycle::State const&) override;
    CallbackReturn on_activate(rclcpp_lifecycle::State const&) override;
    CallbackReturn on_deactivate(rclcpp_lifecycle::State const&) override;
    CallbackReturn on_cleanup(rclcpp_lifecycle::State const&) override;
    CallbackReturn on_shutdown(rclcpp_lifecycle::State const&) override;
    CallbackReturn on_error(rclcpp_lifecycle::State const&) override;
};
