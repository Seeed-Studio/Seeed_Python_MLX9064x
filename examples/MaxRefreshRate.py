import seeed_mlx90640
import time
def main():
    mlx = seeed_mlx90640.grove_mxl90640()
    mlx.refresh_rate = seeed_mlx90640.RefreshRate.REFRESH_8_HZ  # The fastest for raspberry 4 
    frame = [0] * 768
    while True:
        start = time.time()
        try:
            mlx.getFrame(frame)
        except ValueError:
            continue
        print(frame)
        end = time.time()
        print("The time: %f"%(end - start))
if __name__  == '__main__':
    main()