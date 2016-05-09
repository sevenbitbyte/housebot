# vega4_head

Head controller firmware responsible for driving the face LED matrix, ear LED rings, audio amplifier and PIR sensor.

## Flashing

The teensy 3.x can be flashed using [Teensy Loader CLI](https://github.com/PaulStoffregen/teensy_loader_cli)

```
roscd;
cd ..
catkin_make
teensy_loader_cli -mmcu=mk20dx256 -w build/housebot_ros/vega4_head/teensy/bin/head.elf.hex
```

### Subscriptions

 * /head/image [sensor_msgs/Image]
 * /head/left_led_rgb [std_msgs/UInt8MultiArray]
 * /head/right_led_rgb [std_msgs/UInt8MultiArray]

### Publications

 * /head/motion_detected [std_msgs/Bool]

### Parameters
