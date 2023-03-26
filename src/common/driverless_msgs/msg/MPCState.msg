float64 s             # distance along track centerline (used with matching track)
float64 ey            # distance from track centerline
float64 etheta        # angle relative to track centerline
float64 otheta_dot    # angular velocity of the car relative to ground (combine with track curvature to get angular velocity relative to track centerline)
float64 vx            # forward velocity
float64 vy            # sideways (left) velocity
float64 d             # steering angle
builtin_interfaces/Time t                # time
