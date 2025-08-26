from ads1256_quad_api import CoreSystem
import time


def main():
    cs = CoreSystem().create_default(vref_mv=2500)
    cs.start()
    cs.start_thread()
    start = time.time()
    frames = 0
    while time.time() - start < 2.0:  # run 2 seconds
        f = cs.pop_frame()
        if f is not None:
            frames += 1
            if frames % 50 == 0:
                print(
                    f"seq={f['seq']} volts[0:4]={f['volts'][:4]} dropped={cs.get_dropped()}"
                )
        else:
            time.sleep(0.001)
    cs.stop_thread()
    cs.stop()
    print("Total frames:", frames)


if __name__ == "__main__":
    main()
