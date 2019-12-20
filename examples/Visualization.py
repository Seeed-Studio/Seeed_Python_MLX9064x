import seeed_mlx90640
import time
def main():
    sensor = seeed_mlx90640.grove_mxl90640()
    while True:
        Pixel = [0]*801
        for i in range(0,801):
            Pixel[i] = sensor.GetCompensatedPixData(i//32,i%32)
        del Pixel[0:33]
        print(len(Pixel))
        print(Pixel)
if __name__  == '__main__':
    main()