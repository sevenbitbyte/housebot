# vega4_head

Head controller firmware responsible for driving the face LED matrix, ear LED rings, audio amplifier and PIR sensor.

## Flashing


### Subscriptions

 * /belt/pump [std_msgs/Byte]
 * /belt/led_rgbw [std_msgs/UInt8MultiArra]y

### Publications

 * /current [std_msgs/Float32] - RAW units direct from analog read... should eventually be converted to mA
 * /voltage [std_msgs/Float32] - Reported in Volts

### Parameters
