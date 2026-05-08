cmake_minimum_required(VERSION 3.5)
project(mycobot_description)
find_package(ament_cmake REQUIRED)
# Instalando urdf na pasta share/mycobot_description/urdf
install(DIRECTORY urdf DESTINATION share/mycobot_description)
install(PROGRAMS mycobot_description/joint_rebrander.py DESTINATION lib/mycobot_description)
ament_package()
