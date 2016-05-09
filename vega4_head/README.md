# vega4_head

Head controller firmware responsible for driving the face LED matrix, ear LED rings, audio amplifier and PIR sensor.

## Flashing


### Subscriptions

 * /head/image [sensor_msgs/Image]
 * /head/left_ear [std_msgs/UInt8MultiArray]
 * /head/right_ear [std_msgs/UInt8MultiArray]

### Publications

 * /head/motion [std_msgs/Bool]

### Parameters
