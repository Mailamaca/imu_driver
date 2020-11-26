# encoders
ROS Node to interact with encoders

## Initialize ROS2 package

1. Create ros2 C++ package

    ```bash
    ros2 pkg create --build-type ament_cmake --node-name encoders encoders
    ```

2. Build package

    ```bash
    colcon build --packages-select encoders
    ```

3. Source the setup file

    ```bash
    . install/setup.bash
    ```

4. Use the package

    ```bash
    ros2 run encoders encoders
    ```

5. Customize package.xml


## I2C

It connects to i2c with the i2c_server node.

## Hardware

This package interact with an ESP32 board installed on the Maile Vehicle.
The communication is made on I2C where the board has been setup as a SLAVE with ID:0x09.

### Registers to work with

1. Read 0x09

- It will respond with 1 byte, how many encoders are programmed (uint8_t)

1. Read 0x10

- It will respond with (4+2*N_ENCODERS) bytes; first 4 bytes are microsecods from last call (uint32_t) after that, 2 bytes for each programmed encoder counting tick from the last call (uint16_t)
the encoders order is:
1. spur gear motor encoder 
2. front right wheel encoder
3. rear right wheel encoder
4. rear left wheel encoder
5. front left wheel encoder

**NOTES**
- spur gear (high gear ratio = 1:5.22, low gear ratio  = 1:14.45)
- tick at spur gear / round = 10
  
- wheel radious = 0.559 meter
- tick for wheel / round = 12



# Copyright and License

This software contains code licensed as described in LICENSE.