# QUTMS Driverless

## An Autonomous FSAE/FS racecar
The Autonomous team at QUTMS has been in development of an autonomous package for an electric racecar since 2020. Many simulations have been designed or adapted to develop perception and control algorithms while physical hardware was unavailable.

QEV-3D, *Lando*, has been the first testbench for the Autonomous team and was used for hardware testing throughout 2021. As of November 2021, the team had successfully tested perception (LiDAR and camera) and control (steering actuation) on Lando. Additionally, the team took the autonomous package on QEV-3D to track the following month, testing these components with a safety driver to control throttle and brake only, while also gathering an abundance of real-world™️ data. This has allowed further development for real-time pipelines, using machine learning and clustering models to detect cones outlining the track circuit.

In 2022, the team was determined to race QEV-3D as one of the first autonomous racecars in Australia during the 2022 FSAE Australasian Competition. After months of hardware and software debugging, ensuring algorithms were reliable, and redesigning some critical systems, QEV-3D was able to drive itself in October. This was remote controlled at first, then operated under its own processes. Soon after, the team was on the road to the competition in December, where QEV-3D passed all technical inspections to be the second competition-ready FSAE Autonomous Vehicle.

Moving into 2023, determined to improve the reliability of our systems, the team outfitted QEV-3D with a new emergency brake actuator in addition to new EBS control electronics to improve safety systems. The entire navigation and guidance software stack was replaced with "off-the-shelf" ROS 2 libraries from [Nav2](https://github.com/ros-navigation/navigation2), allowing QEV-3D to utilise mature and reliable algorithms for mapping, planning, and control (https://arxiv.org/abs/2311.14276).

For the 2024 season, the goal was the optimisation of all autonomous compute systems in hardware and software. This included streamlining perception pipelines and fleshing out the Nav2 stack, while downsizing and compacting processing devices. These improvements allowed the team to successfully complete QEV-3D's first discovery lap of a competition-spec Trackdrive. Within three months of this achievement, the team had successfully tuned the navigation systems to the point where QEV-3D could complete its first full 10-lap Trackdrive. Looking forward to the Australasian competition, it was the team's best chance in three years to complete the Trackdrive event with QEV-3D being cleared to enter the competition for the final time.

However, disaster struck at the EBS test only minutes before the team's scheduled Trackdrive event. An unpredictable delay in path computing resulted in a sharp correction at 40km/h, just meters before the finish line, veering into the nearby wall and tire barrier. The remote stop and emergency brakes were activated before QEV-3D left the track, and while this reduced the impact speed, there was not enough time to completely stop. Fortunately, the damage was incredibly minor to a single corner's suspension and no stuctural, tractive, or electronic components were affected.

QEV-3D has now retired from competing and will be maintained as an autonomous testbed into 2025 as the next generation of AV is developed by QUTMS.

The Autonomous System is developed using the Robot Operating System (ROS2), for efficient data transfer and package management. QUTMS is constantly pushing the boundaries of cutting-edge to create a sophisticated and future-proof autonomous packages for years to come.
