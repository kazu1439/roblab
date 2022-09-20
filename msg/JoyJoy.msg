std_msgs/Header header  # timestamp in the header is the time the data is received from the joystick
float32[] axes          # the axes measurements from a joystick
int32[] buttons         # the buttons measurements from a joystick
int32[] rise            # the buttons rise from joystick
int32[] fall            # the buttons fall from joystick
int32[] toggle          # the buttons toggle from joystick