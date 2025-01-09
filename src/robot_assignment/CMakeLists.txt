if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add your test file
  ament_add_pytest_test(test_color_3d_detection test/test_color_3d_detection.py)
endif()
