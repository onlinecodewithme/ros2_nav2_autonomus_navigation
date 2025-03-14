# Hardware Setup Guide for Tracked Robot with Jetson Orin NX

This guide explains how to wire and configure the tracked robot with motor controllers using a Jetson Orin NX as the main computing platform.

## Hardware Components

- Jetson Orin NX development kit
- Motor controller board (e.g., Dual H-Bridge motor driver)
- Tracked robot chassis with DC motors
- Power supply system (batteries)
- ZED stereo camera
- Additional sensors (optional)
- Wiring and connectors

## Jetson Orin NX Overview

The NVIDIA Jetson Orin NX provides:
- GPIO pins for motor control signals
- USB ports for connecting the ZED camera
- Ethernet/Wi-Fi for network communication
- Multiple I2C, SPI, and UART interfaces for additional sensors

## Wiring Diagram

```
+---------------+           +--------------------+
| Jetson Orin NX|           | Motor Controller   |
|               |           | (Dual H-Bridge)    |
|          GPIO |---------->| IN1, IN2, IN3, IN4 |
|               |           |                    |
|           PWM |---------->| ENA, ENB           |
|               |           |                    |
|          3.3V |---------->| VCC (Logic)        |
|               |           |                    |
|          GND  |---------->| GND                |
+---------------+           |                    |
                            |              OUT1 |----> Left Track Motor +
  Battery +                 |              OUT2 |----> Left Track Motor -
  (Logic) |---------------->| VCC (Logic)       |
                            |                    |
                            |              OUT3 |----> Right Track Motor +
  Battery +                 |              OUT4 |----> Right Track Motor -
  (Motor) |---------------->| VCC (Motor)       |
                            |                    |
  Battery - |---------------->| GND              |
            |                 +--------------------+
            |
            +-------> Jetson Orin NX GND
```

## GPIO Pin Connections

| Jetson Orin NX                | Motor Controller              | Function              |
|-------------------------------|-------------------------------|-----------------------|
| GPIO Pin (e.g., GPIO12)       | IN1                          | Left Motor Direction 1 |
| GPIO Pin (e.g., GPIO13)       | IN2                          | Left Motor Direction 2 |
| GPIO Pin (e.g., GPIO16)       | IN3                          | Right Motor Direction 1|
| GPIO Pin (e.g., GPIO19)       | IN4                          | Right Motor Direction 2|
| PWM Pin (e.g., GPIO32)        | ENA                          | Left Motor Speed       |
| PWM Pin (e.g., GPIO33)        | ENB                          | Right Motor Speed      |
| 3.3V                          | VCC (Logic)                  | Logic Power            |
| GND                           | GND                          | Ground                 |

**Note**: The actual GPIO pin numbers will depend on your specific Jetson Orin NX carrier board. Refer to the Jetson Orin NX pinout diagram for your specific model.

## Power System

1. **Logic Power Supply**:
   - Connect a regulated 5V supply to the motor controller's logic VCC
   - This can come from the Jetson Orin NX if the current requirements are low, or use a separate regulator

2. **Motor Power Supply**:
   - Use a separate battery (typically 7.4V-12V LiPo) for powering the motors
   - Connect to the motor controller's motor VCC
   - Ensure the battery can handle the current demands of your motors

3. **Jetson Orin NX Power**:
   - Power the Jetson Orin NX using its dedicated power input
   - Typically requires a 9V-20V DC supply depending on the carrier board

4. **Common Ground**:
   - Make sure all ground connections are connected together
   - Both power supplies and the Jetson Orin NX should share a common ground

## Motor Controller Configuration

### For Dual H-Bridge Controllers (e.g., L298N, TB6612FNG):

1. **Direction Control**:
   - IN1 & IN2 control left track direction
   - IN3 & IN4 control right track direction
   - Logic patterns:
     - Forward: IN1=HIGH, IN2=LOW (left) / IN3=HIGH, IN4=LOW (right)
     - Backward: IN1=LOW, IN2=HIGH (left) / IN3=LOW, IN4=HIGH (right)
     - Stop: IN1=LOW, IN2=LOW (left) / IN3=LOW, IN4=LOW (right)

2. **Speed Control**:
   - ENA controls the left track speed via PWM
   - ENB controls the right track speed via PWM
   - PWM frequency: typically 20-25 kHz works well for DC motors

## Software Configuration

### Enabling GPIO and PWM on Jetson Orin NX

1. **Install required tools**:
   ```bash
   sudo apt-get update
   sudo apt-get install python3-pip
   sudo pip3 install Jetson.GPIO
   ```

2. **Enable I2C permissions** (if using I2C sensors):
   ```bash
   sudo usermod -aG i2c $USER
   ```

3. **Enable GPIO permissions**:
   ```bash
   sudo groupadd -f -r gpio
   sudo usermod -a -G gpio $USER
   sudo cp /opt/nvidia/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

### ROS 2 Configuration

1. **Create a motor driver node**:
   Create a new ROS 2 package for hardware interface:

   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_cmake jetson_motor_driver --dependencies rclcpp std_msgs
   ```

2. **Implement a node to interface with GPIO**:
   Create a new C++ or Python node that subscribes to the `/left_track_cmd` and `/right_track_cmd` topics and controls the GPIO pins accordingly.

   Example for Python (place in the `jetson_motor_driver/jetson_motor_driver/` directory):
   
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Int32
   import Jetson.GPIO as GPIO
   
   class MotorDriverNode(Node):
       def __init__(self):
           super().__init__('motor_driver_node')
           
           # Configure GPIO pins
           self.left_dir_pin1 = 12  # Adjust to your GPIO pin
           self.left_dir_pin2 = 13  # Adjust to your GPIO pin
           self.right_dir_pin1 = 16  # Adjust to your GPIO pin
           self.right_dir_pin2 = 19  # Adjust to your GPIO pin
           self.left_pwm_pin = 32  # Adjust to your GPIO pin
           self.right_pwm_pin = 33  # Adjust to your GPIO pin
           
           # Set up GPIO
           GPIO.setmode(GPIO.BOARD)
           GPIO.setup(self.left_dir_pin1, GPIO.OUT)
           GPIO.setup(self.left_dir_pin2, GPIO.OUT)
           GPIO.setup(self.right_dir_pin1, GPIO.OUT)
           GPIO.setup(self.right_dir_pin2, GPIO.OUT)
           GPIO.setup(self.left_pwm_pin, GPIO.OUT)
           GPIO.setup(self.right_pwm_pin, GPIO.OUT)
           
           # Set up PWM
           self.left_pwm = GPIO.PWM(self.left_pwm_pin, 20000)  # 20kHz frequency
           self.right_pwm = GPIO.PWM(self.right_pwm_pin, 20000)  # 20kHz frequency
           self.left_pwm.start(0)
           self.right_pwm.start(0)
           
           # Create subscribers
           self.left_sub = self.create_subscription(
               Int32,
               'left_track_cmd',
               self.left_track_callback,
               10)
               
           self.right_sub = self.create_subscription(
               Int32,
               'right_track_cmd',
               self.right_track_callback,
               10)
           
           self.get_logger().info('Motor driver node initialized')
       
       def left_track_callback(self, msg):
           pwm_value = msg.data
           
           # Set direction based on sign
           if pwm_value >= 0:
               GPIO.output(self.left_dir_pin1, GPIO.HIGH)
               GPIO.output(self.left_dir_pin2, GPIO.LOW)
               self.left_pwm.ChangeDutyCycle(min(abs(pwm_value), 100))
           else:
               GPIO.output(self.left_dir_pin1, GPIO.LOW)
               GPIO.output(self.left_dir_pin2, GPIO.HIGH)
               self.left_pwm.ChangeDutyCycle(min(abs(pwm_value), 100))
       
       def right_track_callback(self, msg):
           pwm_value = msg.data
           
           # Set direction based on sign
           if pwm_value >= 0:
               GPIO.output(self.right_dir_pin1, GPIO.HIGH)
               GPIO.output(self.right_dir_pin2, GPIO.LOW)
               self.right_pwm.ChangeDutyCycle(min(abs(pwm_value), 100))
           else:
               GPIO.output(self.right_dir_pin1, GPIO.LOW)
               GPIO.output(self.right_dir_pin2, GPIO.HIGH)
               self.right_pwm.ChangeDutyCycle(min(abs(pwm_value), 100))
           
       def __del__(self):
           # Clean up GPIO
           self.left_pwm.stop()
           self.right_pwm.stop()
           GPIO.cleanup()

   def main(args=None):
       rclpy.init(args=args)
       node = MotorDriverNode()
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass
       finally:
           node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. **Update package configuration**:
   Modify your `setup.py` or `CMakeLists.txt` to include the motor driver node.

4. **Launch integration**:
   Update the launch files to include the motor driver node:

   ```python
   # Add to robot_complete.launch.py
   Node(
       package='jetson_motor_driver',
       executable='motor_driver_node',
       name='motor_driver_node',
       output='screen'
   ),
   ```

## Calibration and Testing

1. **Motor Direction Test**:
   ```bash
   # Test left motor
   ros2 topic pub --once /left_track_cmd std_msgs/msg/Int32 "{data: 50}"
   
   # Test right motor
   ros2 topic pub --once /right_track_cmd std_msgs/msg/Int32 "{data: 50}"
   ```

2. **Speed Calibration**:
   You may need to adjust the PWM values in the `cmd_vel_to_tracks.cpp` node to match your specific motors:
   
   ```bash
   # Edit the conversion factor in the tracked_robot_controller package
   ros2 param set /cmd_vel_to_tracks max_pwm 200  # Adjust if 255 is too fast
   ```

## Troubleshooting

### Motor Not Responding
- Check wiring connections between Jetson and motor controller
- Verify GPIO pin numbers match your code
- Ensure the motor power supply is connected and charged
- Check for any error messages in the motor driver node

### Motors Running in Wrong Direction
- Swap the motor wires (OUT1/OUT2 or OUT3/OUT4) at the motor controller
- Or modify the GPIO logic in the motor driver code

### Erratic Motor Behavior
- Check for loose connections
- Ensure the ground connections are solid
- Add capacitors across motor terminals to reduce noise (0.1μF ceramic capacitors)

### PWM/GPIO Issues
- Make sure the user has proper permissions for GPIO access
- Verify that the GPIO pins support PWM functionality
- Consider using a dedicated PWM controller if precise speed control is required

## Additional Hardware Notes

### ZED Camera Connection
- Connect the ZED camera to a USB 3.0 port on the Jetson Orin NX
- Mount the camera securely on the robot chassis
- Ensure the camera is positioned to properly view the environment

### Power Considerations
- The Jetson Orin NX can consume 10-25W depending on workload
- Motors can draw several amps, especially during startup
- Use adequately sized batteries (e.g., 5000mAh+ LiPo)
- Consider a separate voltage regulator for stable power to the Jetson

### Cooling
- The Jetson Orin NX may require active cooling for sustained operation
- Install a fan or heatsink as needed
- Monitor temperatures during operation
