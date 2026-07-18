"""kerala_bot_sim: per-axis stepper+belt dynamics simulator for trajectory feasibility.

Pure-Python core (params, model, trajectory, simulate, overlay_plot) has no ROS
dependency and runs anywhere numpy/scipy/matplotlib are installed. The rviz_player
module additionally needs rclpy and is only imported when used.
"""
