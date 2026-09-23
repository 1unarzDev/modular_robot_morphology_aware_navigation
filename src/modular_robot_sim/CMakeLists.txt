cmake_minimum_required(VERSION 3.16)
project(modular_robot_sim)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})

# The fiducial station (ADR 0006) is a simulator sensor model, so it lives on
# the simulation side rather than in an autonomy package.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/fiducial_station_node.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME fiducial_station)

# DART merges fixed-joint children into one skeleton. Generate unique internal
# link and joint names so six instances do not collide when they are redocked.
set(GENERATED_MODELS "${CMAKE_CURRENT_BINARY_DIR}/generated_models")
foreach(POD_INDEX RANGE 0 5)
  set(POD_ID "pod_${POD_INDEX}")
  set(POD_MODEL "drive_pod_${POD_INDEX}")
  set(POD_DIRECTORY "${GENERATED_MODELS}/${POD_MODEL}")
  file(MAKE_DIRECTORY "${POD_DIRECTORY}")
  configure_file(models/drive_pod/model.sdf.in "${POD_DIRECTORY}/model.sdf" @ONLY)
  configure_file(models/drive_pod/model.config.in "${POD_DIRECTORY}/model.config" @ONLY)
  set(POD_MODEL "articulated_pod_${POD_INDEX}")
  set(POD_DIRECTORY "${GENERATED_MODELS}/${POD_MODEL}")
  file(MAKE_DIRECTORY "${POD_DIRECTORY}")
  configure_file(models/articulated_pod/model.sdf.in "${POD_DIRECTORY}/model.sdf" @ONLY)
  configure_file(models/drive_pod/model.config.in "${POD_DIRECTORY}/model.config" @ONLY)
endforeach()
install(DIRECTORY "${GENERATED_MODELS}/" DESTINATION share/${PROJECT_NAME}/models)
ament_package()
