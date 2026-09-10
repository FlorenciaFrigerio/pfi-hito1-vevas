cmake_minimum_required(VERSION 3.8)
project(irb120_description)

find_package(ament_cmake REQUIRED)

# Instala los directorios de datos en el share del paquete.
# Sin esto, package://irb120_description/... NO resuelve en RViz.
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Scripts de generacion y validacion (ejecutables)
install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
