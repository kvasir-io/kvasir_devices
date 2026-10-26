# Input devices without a bus: the quadrature decoder of the rotary encoders, the edge-counting RotaryEncoder,
# PushButton in both modes.

kvasir_devices_test(input_quadrature_test SOURCE quadrature_test.cpp)
kvasir_devices_test(input_rotary_encoder_test SOURCE rotary_encoder_test.cpp)
kvasir_devices_test(input_push_button_test SOURCE push_button_test.cpp)
