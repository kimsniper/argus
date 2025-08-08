idf_component_register(
    SRCS "pwm_hal.cpp" "pwm.cpp" "servo.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver
)