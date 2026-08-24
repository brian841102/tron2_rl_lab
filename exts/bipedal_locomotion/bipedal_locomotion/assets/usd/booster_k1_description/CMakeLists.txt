cmake_minimum_required(VERSION 3.8)
project(booster_k1_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    description_tests
    test/test_description.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120
  )
endif()

ament_package()
