set(_patch_dir "${CMAKE_CURRENT_LIST_DIR}")

function(_apply_patch path)
  execute_process(
      COMMAND patch -p1 -N -r - -i "${path}"
      RESULT_VARIABLE _res)
  if(NOT (_res EQUAL 0 OR _res EQUAL 1))
    message(FATAL_ERROR "AimRT patch failed (${_res}): ${path}")
  endif()
endfunction()

_apply_patch("${_patch_dir}/aimrt_ros2_typesupport_rmw_fix.patch")
_apply_patch("${_patch_dir}/aimrt_ros2_serialize_buffer_fix.patch")
