cmake_minimum_required(VERSION 3.8)
project(my_sample_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# ------------------------------------------------------------------------------
# 🪄 마법의 코드: scripts 폴더 안의 모든 .py 파일을 자동으로 찾아서 설치합니다.
# ------------------------------------------------------------------------------
file(GLOB python_scripts "scripts/*.py")

install(PROGRAMS
  ${python_scripts}
  DESTINATION lib/${PROJECT_NAME}
)
# ------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
