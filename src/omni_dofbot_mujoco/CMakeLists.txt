cmake_minimum_required(VERSION 3.8)
project(omni_dofbot_mujoco)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/mujoco_bridge_node.py
  scripts/_path.py
  scripts/build_model.py
  scripts/run_acceptance_tests.py
  scripts/friction_sweep.py
  scripts/plano_inclinado.py
  scripts/view.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
