# seeed MLX9064x

The MLX9064x is a fully calibrated 32x24(12 x 16) pixels IR array in an industry standard 4-lead TO39 package with digital interface The MLX90640 contains 768 FIR pixels and The MLX90641 contains 192 FIR pixels. An ambient sensor is integrated to measure the ambient temperature of the chip and supply sensor to measure the VDD. The outputs of all sensors IR, Ta and VDD are stored in internal RAM and are accessible through I2C.

# Dependencies

This driver depends on:

- [***grove.py***](https://github.com/Seeed-Studio/grove.py)

This is easy to install with the following command.

```
pip3 install Seeed-grove.py
```
 
## Installing from PyPI

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally from PyPI. To install for current user:

```
pip3 install seeed-python-mlx9064x
```

To install system-wide (this may be required in some cases):

```
sudo pip3 install seeed-python-mlx9064x
```

if you want to update the driver locally from PyPI. you can use:

```
pip3 install --upgrade seeed-python-mlx9064x
```

## Usage Notes

First, Check the corresponding i2c number of the board:

```
pi@raspberrypi:~ $ ls /dev/i2c*
/dev/i2c-1
```

Check if the i2c device works properly， 0x33 is the MLX9064x i2c address.

```
pi@raspberrypi:~/Seeed_Python_SGP30 $ i2cdetect -y -r 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- 33 -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

## initialize the sersor object:

Initialize the sersor object and config the sersor refresh rate.

```python
import seeed_mlx9064x
mlx = seeed_mlx9064x.grove_mxl90640()
#mlx = seeed_mlx9064x.grove_mxl90641()
mlx.refresh_rate = seeed_mlx9064x.RefreshRate.REFRESH_8_HZ  # The fastest for raspberry 4 
# REFRESH_0_5_HZ = 0b000  # 0.5Hz
# REFRESH_1_HZ = 0b001  # 1Hz
# REFRESH_2_HZ = 0b010  # 2Hz
# REFRESH_4_HZ = 0b011  # 4Hz
# REFRESH_8_HZ = 0b100  # 8Hz
# REFRESH_16_HZ = 0b101  # 16Hz
# REFRESH_32_HZ = 0b110  # 32Hz
# REFRESH_64_HZ = 0b111  # 64Hz
```

## Reading from the Sensor

To read from the sensor:

```python
     try:
     
          frame = [0]*768
          #frame = [0]*192
          mlx.getFrame(frame)
     except ValueError:
          continue
```

maybe you can add content that below to the config.txt to get the fastest rate recommended for compatibility

```bash
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```  

This will give you a framerate of - at most - 8FPS.

## Contributing

If you have any good suggestions or comments, you can send issues or PR us.
