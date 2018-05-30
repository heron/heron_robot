HERON_CONTROLLER_CONFIG=$(catkin_find --etc --first-only heron_control heron_controller.yaml 2>/dev/null)
if [ -z "$HERON_CONTROLLER_CONFIG" ]; then
  HERON_CONTROLLER_CONFIG=$(catkin_find --share --first-only heron_control config/heron_controller.yaml 2>/dev/null)
fi

export HERON_CONTROLLER_CONFIG
