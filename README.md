# Prismis™

CPU Path Tracing demo written in C!

## Building & running

Build and run with:

```shell
make && ./build/prismis
```

You can tweak the following constants in the source code to adjust rendering:

```c
#define WIDTH 960
#define HEIGHT 540
#define MAX_DEPTH 5 // Maximum number of bounces per ray
#define SAMPLES 4 // Number of samples per pixel
#define THREADS 8
#define SEGMENT_SIZE 16 // Size of image segment (tile) for work distribution among threads
```

## Gallery
<details>
<summary>Expand</summary>
<img src="16k_200samples_20depth.png">
</details>
