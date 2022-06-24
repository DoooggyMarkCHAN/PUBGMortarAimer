import keyboard
import time
import win32api
import win32con
import math
from pynput import mouse
import pyttsx3

# Velocity of Projectile
V = 117.2
# pi
pi = 3.1415926535
# Scroll Displacement
Scroll_D = 9550
# Zoom Factor
ZoomFactor = 1
# Measuring Point
point_1 = [0, 0]
point_2 = [0, 0]
# Matching Point
match_1 = [0, 0]
match_2 = [0, 0]

# Measuring / Matching mode switch parameters
isMeasuring = False

# index of inputting point
m_point = 1

# Pixel distance % Real Distance
mDistance = 0
realDistance = 0

# Conversion Scale
scale = 0
print('After Pressing M to open the map')
print('Press CTRL + Left Mouse Key to match and measure')
print('Matching is required every time when the map is opened')

point_2 = [960, 540]


def on_scroll(x, y, dx, dy):
    global ZoomFactor
    global isMeasuring
    if isMeasuring:
        if dy > 0 and isMeasuring:
            print('Up')
            if ZoomFactor < 5:
                ZoomFactor = ZoomFactor + 1
        elif dy < 0 and isMeasuring:
            print('Down')
            if ZoomFactor > 1:
                ZoomFactor = ZoomFactor - 1
        print(ZoomFactor)


LS = mouse.Listener(on_scroll=on_scroll)
LS.start()

engine = pyttsx3.init()

while 1:

    if keyboard.is_pressed('down'):
        while keyboard.is_pressed('down'):
            continue
        win32api.mouse_event(win32con.MOUSEEVENTF_WHEEL, 0, 0, -Scroll_D, 0)

    if keyboard.is_pressed('up') and not isMeasuring:
        while keyboard.is_pressed('up'):
            continue
        if 121 < realDistance < 700:
            if 500 < realDistance < 700:
                realDistance = realDistance - 1
            elif 400 < realDistance < 500:
                realDistance = realDistance - 2
            elif 300 < realDistance < 400:
                realDistance = realDistance - 3
            elif 200 < realDistance < 300:
                realDistance = realDistance - 4
            elif 121 < realDistance < 200:
                realDistance = realDistance - 5

            DEG = (pi - math.asin(2 * 9.81 * realDistance / (V * V))) / 2
            # print(DEG)
            Scroll_d = int(round((DEG - pi / 4) / (2 / 9 * pi) * Scroll_D, 0))
            # print(Scroll_d)
            win32api.mouse_event(win32con.MOUSEEVENTF_WHEEL, 0, 0, Scroll_d, 0)

    if keyboard.is_pressed('esc'):
        while keyboard.is_pressed('esc'):
            continue
        if isMeasuring:
            isMeasuring = False
            print('Measuring Off')
            m_point = 1
    # -----------------------------------------------------------------------------------
    # ----------------------------Map Measure-------------------------------------------
    if keyboard.is_pressed('m'):
        while keyboard.is_pressed('m'):
            continue
        if not keyboard.is_pressed('m'):
            if not isMeasuring:
                isMeasuring = True
                print('Measuring On')
                m_point = 1
            else:
                isMeasuring = False
                print('Measuring Off')

    # Click to input 1st point and 2nd point of matching
    if (win32api.GetAsyncKeyState(0x02) & 0x8000) and isMeasuring:

        if ZoomFactor == 1:
            scale = 100 / 13.5
        elif ZoomFactor == 2:
            scale = 100 / 28.4
        elif ZoomFactor == 3:
            scale = 100 / 54.1
        elif ZoomFactor == 4:
            scale = 100 / 108
        elif ZoomFactor == 5:
            scale = 100 / 216

        point_1 = win32api.GetCursorPos()
        print(point_1)

        pixelDistance = math.sqrt(
            (point_2[0] - point_1[0]) * (point_2[0] - point_1[0]) + (point_2[1] - point_1[1]) * (
                    point_2[1] - point_1[1]))
        realDistance = pixelDistance * scale
        realDistance_str = str(int(realDistance))
        print('Real Distance:', realDistance_str)
        engine.say(realDistance_str)
        engine.runAndWait()
        point_1 = [0, 0]
    time.sleep(0.1)
