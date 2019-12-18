# seeed MLX90640

The MLX90640 is a fully calibrated 32x24 pixels IR array in an industry standard 4-lead TO39 package with digital interface The MLX90640 contains 768 FIR pixels. An ambient sensor is integrated to measure the ambient temperature of the chip and supply sensor to measure the VDD. The outputs of all sensors IR, Ta and VDD are stored in internal RAM and are accessible through I2C.

# Dependencies

This driver depends on:
- [***grove.py***](https://github.com/Seeed-Studio/grove.py)

This is easy to install with the following command.
 ```
curl -sL https://github.com/Seeed-Studio/grove.py/raw/master/install.sh | sudo bash -s -
 ```
## Installing from PyPI

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally from PyPI. To install for current user:

```
pip3 install seeed-python-mlx90640
```

To install system-wide (this may be required in some cases):

```
sudo pip3 install seeed-python-mlx90640
```

## Usage Notes

First, Check the corresponding i2c number of the board:
```
(.env) pi@raspberrypi:~ $ ls /dev/i2c*
/dev/i2c-1
```

Check if the i2c device works properly， 0x33 is the MLX90640 i2c address.
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

Next, initialize the sersor object:

```python
import seeed_mlx90640
sensor = seeed_mlx90640.grove_mxl90640() 
```

## Reading from the Sensor

To read from the sensor:

```python
Pixel = [0]*801
for i in range(0,801):
     Pixel[i] = sensor.getCompensatedPixData(i//32,i%32)
del Pixel[0:33]
print(len(Pixel))  #24x32 pixel
print(Pixel)
```

## Contributing

If you have any good suggestions or comments, you can send issues or PR us.
