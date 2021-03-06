# vega4_belt

Belt controller firmware responsible for driving the face LED matrix, ear LED rings, audio amplifier and PIR sensor.

## Flashing

The teensy 3.x can be flashed using [Teensy Loader CLI](https://github.com/PaulStoffregen/teensy_loader_cli)

```
roscd;
cd ..
catkin_make
teensy_loader_cli -mmcu=mk20dx256 -w build/housebot_ros/vega4_belt/teensy/bin/belt.elf.hex
```

### Subscriptions

 * /belt/pump [std_msgs/Byte]
 * /belt/belt_led_rgbw [std_msgs/UInt8MultiArra]y

### Publications

 * /current [std_msgs/Float32] - RAW units direct from analog read... should eventually be converted to mA
 * /voltage [std_msgs/Float32] - Reported in Volts

### Parameters
