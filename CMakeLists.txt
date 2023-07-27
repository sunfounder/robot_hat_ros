find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${robot_hat_ros}
  "srv/RegisterADC.srv"
  "srv/RemoveADC.srv"
)