import seeed_mlx9064x
import time
CHIP_TYPE = 'MLX90641'
def main():
    if CHIP_TYPE == 'MLX90641':
        mlx = seeed_mlx9064x.grove_mxl90641()
        frame = [0] * 192
    elif CHIP_TYPE == 'MLX90640':
        mlx = seeed_mlx9064x.grove_mxl90640()
        frame = [0] * 768  
    mlx.refresh_rate = seeed_mlx9064x.RefreshRate.REFRESH_8_HZ  # The fastest for raspberry 4 
    time.sleep(1)
    while True:
        start = time.time()
        mlx.getFrame(frame)
        print(frame)
        end = time.time()
        print("The time: %f"%(end - start))
if __name__  == '__main__':
    main()