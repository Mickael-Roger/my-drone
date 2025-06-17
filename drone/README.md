# ICM20948

- Issue with bad espressif component referencement: https://github.com/cybergear-robotics/icm20948/issues/6
  Temporary solution: `idf.py add-dependency --git https://github.com/cybergear-robotics/icm20948 "cybergear-robotics/icm20948"`

- ICM20948 does not compile because of spi error in code
  Temporary solution: Remove icm20948_spi.c from  `managed_components/cybergear-robotics__icm20948/CMakeLists.txt`

