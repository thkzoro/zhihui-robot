#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)


SENSORS_DIR = gesture/paj7620 \
            imu/mpu6050 \
            sensor_hub 
           

COMPONENT_ADD_INCLUDEDIRS := . sensor_hub/include $(SENSORS_DIR) 
COMPONENT_SRCDIRS := . $(SENSORS_DIR) 
