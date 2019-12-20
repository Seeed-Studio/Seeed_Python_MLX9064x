import seeed_mlx90640
import time
def main():
    sensor = seeed_mlx90640.grove_mxl90640()
    sensor.SetRefreshRate(0x04)
    # 0x00 0.5HZ
    # 0x01 1HZ
    # 0x02 2HZ
    # 0x03 4HZ(recommend for raspberry)
    # 0x04 8HZ
    # 0x05 16HZ
    # 0x06 32HZ
    # 0x07 64HZ
    while True:
        Pixel = [0]*801
        for i in range(0,801):
            Pixel[i] = sensor.GetCompensatedPixData(i//32,i%32)
            print(Pixel[i])
if __name__  == '__main__':
    main()