
set(SENSORS_DIR "gesture/paj7620"
            "imu/mpu6050"
            "sensor_hub"
            )

idf_component_register(SRC_DIRS "${SENSORS_DIR}" "."
                        INCLUDE_DIRS "${SENSORS_DIR}" "." "sensor_hub/include"
                        PRIV_INCLUDE_DIRS "."
                        REQUIRES bus
                        )
