cmake_minimum_required(VERSION 3.5)
project(insta360_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(imu_tools REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_search_module(SWSCALE REQUIRED libswscale)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# Insta360 SDK
#   CameraSDK 2.1.8 : 本パッケージ同梱 (include/camera, include/stream, lib/)
#   MediaSDK  3.1.5 : deb でシステム導入 (/opt/MediaSDK-3.1.5-linux/)
#                     ヘッダのみ include/media/ に同梱
#
# 旧 MediaSDK 3.1.1 (libmediasdk-dev) がシステムに残っていても問題ない:
#   ヘッダ … /usr/include は -I パスではなく常に最後に探索されるため include/media が必ず勝つ
#   ライブラリ … 絶対パスでリンクし、RPATH で /opt 側を先に見せる
# ---------------------------------------------------------------------------
set(CAMERA_SDK_LIB "${PROJECT_SOURCE_DIR}/lib/libCameraSDK.so")
if(NOT EXISTS "${CAMERA_SDK_LIB}")
  message(FATAL_ERROR "CameraSDK が見つかりません: ${CAMERA_SDK_LIB}\n"
                      "  CameraSDK 2.1.8 の libCameraSDK.so を lib/ に、"
                      "camera/stream ヘッダを include/ に配置してください (README 参照)")
endif()

set(MEDIA_SDK_DIR "/opt/MediaSDK-3.1.5-linux" CACHE PATH "MediaSDK 3.1.5 install prefix")
if(NOT EXISTS "${MEDIA_SDK_DIR}/lib/libMediaSDK.so")
  message(FATAL_ERROR "MediaSDK が見つかりません: ${MEDIA_SDK_DIR}/lib/libMediaSDK.so\n"
                      "  MediaSDK-3.1.5-linux-amd64.deb を dpkg -i で導入してください")
endif()

# libMediaSDK.so は jpeg_* を多数グローバルに export しているため、libjpeg を MediaSDK より
# 先に DT_NEEDED へ登録してシンボル解決順で勝たせる (負けると本ノードの jpeg_* 呼び出しが
# MediaSDK 内蔵のコーデックへ飛ぶ)。
find_library(SYS_JPEG NAMES jpeg PATHS /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH REQUIRED)

# RPATH の順序が重要: ${MEDIA_SDK_DIR}/lib にも別ビルドの libCameraSDK.so (20MB) があるため、
# 同梱版 (17MB) を必ず先に置く。colcon --symlink-install ではバイナリ実体が build/ 以下に
# 置かれるため $ORIGIN は使えず、絶対パスで焼き込む。
set(INSTA360_RPATH "${PROJECT_SOURCE_DIR}/lib;${MEDIA_SDK_DIR}/lib")

install(
  DIRECTORY config launch lib
  DESTINATION share/${PROJECT_NAME}
)

add_executable(${PROJECT_NAME} src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  camera_info_manager
  cv_bridge
  image_transport
  std_msgs
  std_srvs
  OpenCV
)
target_link_libraries(${PROJECT_NAME} ${CAMERA_SDK_LIB}  ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "${INSTA360_RPATH}"
  BUILD_WITH_INSTALL_RPATH TRUE)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(decoder src/decoder.cpp)
ament_target_dependencies(decoder
  rclcpp
  camera_info_manager
  cv_bridge
  image_transport
  std_msgs
  std_srvs
  OpenCV
)
# decoder は Insta360 SDK の API を一切使わない (ffmpeg でデコードするだけ) ので CameraSDK はリンクしない。
# MediaSDK もリンクしないため、こちらは OpenCV/cv_bridge を安全に使える。
target_link_libraries(decoder ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})

install(TARGETS decoder
  DESTINATION lib/${PROJECT_NAME}
)

# Integrated node (raw H.264/H.265 + IMU + MediaSDK panorama, single camera open)
#
# OpenCV は意図的にリンクしない。${MEDIA_SDK_DIR}/lib/libopencv_world.so.407 (2.2GB) が
# 同梱されており、システム OpenCV を同じプロセスに入れると main() 到達前に SIGSEGV する。
# そのため JPEG エンコードは cv::imencode ではなく libjpeg 直叩き (RGBA -> JPEG) で行う。
add_executable(insta360_all_node src/insta360_all_node.cpp)
ament_target_dependencies(insta360_all_node
  rclcpp
  sensor_msgs
)
# 同梱SDKのヘッダを、システムに残っている旧 /usr/include/ins_*.h より先に見せる。
target_include_directories(insta360_all_node BEFORE PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/media)
target_link_libraries(insta360_all_node
  -Wl,--no-as-needed ${SYS_JPEG} -Wl,--as-needed
  ${CAMERA_SDK_LIB}
  ${MEDIA_SDK_DIR}/lib/libMediaSDK.so)
set_target_properties(insta360_all_node PROPERTIES
  INSTALL_RPATH "${INSTA360_RPATH}"
  BUILD_WITH_INSTALL_RPATH TRUE)
install(TARGETS insta360_all_node
  DESTINATION lib/${PROJECT_NAME}
)

# Add C++ equirectangular node
add_executable(equirectangular_cpp src/equirectangular.cpp)
ament_target_dependencies(equirectangular_cpp
  rclcpp
  cv_bridge
  OpenCV
)
target_link_libraries(equirectangular_cpp ${OpenCV_LIBRARIES})

install(TARGETS equirectangular_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/equirectangular.py
  scripts/calibrate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
