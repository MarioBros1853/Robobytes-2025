import sensor
from pyb import UART
import time

uart = UART(1,9600)
uart.init(9600,bits=8)

sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time=2000)  # Wait for settings take effect.

ColorAdolfo = [(0, 29, -128, 127, -128, 127)]
Rojo = [(25, 56, 19, 53, -7, 55)]
Verde = [(26, 74, -6, -53, 56, -61)]
Amarillo = [(46, 97, -128, 127, 16, 127)]
colorDetectionTimer = 0;

H = [[True, False, True], [True, True, True], [True, False, True]]
U = [[True, False, True], [True, False, True], [True, True, True]]
S = [[True, True, True], [True, True, True], [True, True, True]]

while True:
    ms = time.ticks_ms()
    img = sensor.snapshot()  # Take a picture and return the image.

    blobs_rojo = img.find_blobs(Rojo, area_threshold=10000)
    blobs_verde = img.find_blobs(Verde,area_threshold=10000)
    blobs_amarillo = img.find_blobs(Amarillo,area_threshold=10000)
    blobs_negro = img.find_blobs(ColorAdolfo,area_threshold=10000)

    if (blobs_rojo and ms > colorDetectionTimer):
        MaxColor = max(blobs_rojo,key=lambda b: b.pixels())
        img.draw_rectangle(MaxColor.x(),MaxColor.y(),MaxColor.w(),MaxColor.h(),(255,0,0),2,fill=True)
        colorDetectionTimer = ms + 4000;
        uart.write("2")
    elif (blobs_amarillo and ms > colorDetectionTimer):
        MaxColor = max(blobs_amarillo,key=lambda b: b.pixels())
        img.draw_rectangle(MaxColor.x(),MaxColor.y(),MaxColor.w(),MaxColor.h(),(255,255,0),2,fill=True)
        colorDetectionTimer = ms + 3000;
        uart.write("1")
    elif (blobs_verde and ms > colorDetectionTimer):
        MaxColor = max(blobs_verde,key=lambda b: b.pixels())
        img.draw_rectangle(MaxColor.x(),MaxColor.y(),MaxColor.w(),MaxColor.h(),(0,255,0),2,fill=True)
        colorDetectionTimer = ms + 3000;
        uart.write("0")
    elif (blobs_negro):
         print("z")
    else:
        uart.write("9")
