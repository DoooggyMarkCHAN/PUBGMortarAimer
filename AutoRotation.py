# With OpenCV----AutoAiming
import numpy as np
import cv2
from PIL import ImageGrab
import time
import win32api
import win32con
from pynput import mouse
from pynput import keyboard
import pyttsx3
import math
import PySimpleGUI as sg
from mss import mss

w_scr = 2560
h_scr = 1440

# Player Index
player_index = 1
# Map Size
MapSize = "8x8"
# Velocity of Projectile
V = 117.2
# pi
pi = math.pi
# Scroll Displacement
Scroll_D = 9550
# Zoom Factor
ZoomFactor = 1

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
print('Press Space Key to and measure')


def auto_distance_measure(k_scale):
    global w_scr, h_scr
    global player_index
    global MapSize
    if MapSize == "8x8":
        map_scale = 1
    elif MapSize == "6x6":
        map_scale = 0.75
    elif MapSize == "4x4":
        map_scale = 0.5
    elif MapSize == "3x3":
        map_scale = 0.375
    elif MapSize == "2x2":
        map_scale = 0.25

    full_image = ImageGrab.grab()
    w_scr = full_image.width
    h_scr = full_image.height
    frame = np.array(full_image)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(np.array(frame), cv2.COLOR_BGR2HSV)
    if player_index == 1:  # Yellow
        lower_h = 28
        higher_h = 32
    elif player_index == 2:  # Orange
        lower_h = 6
        higher_h = 14
    elif player_index == 3:  # Blue
        lower_h = 95
        higher_h = 105
    elif player_index == 4:  # Green
        lower_h = 55
        higher_h = 65

    lower_range = np.array([lower_h, 160, 160])
    upper_range = np.array([higher_h, 255, 255])
    # Yellow 25~35; Orange 10~20
    mask = cv2.inRange(hsv, lower_range, upper_range)

    result = cv2.bitwise_and(frame, frame, mask=mask)
    result[int(h_scr/2):int(h_scr), 0:int(h_scr*0.16), :] = [0, 0, 0]
    result[int(0.9*h_scr):int(h_scr), 0:w_scr, :] = [0, 0, 0]
    kernel = np.ones((3, 3), np.uint8)
    result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel)
    img_bgr = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)
    img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    contours, _ = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours_effective = []

    for i in range(0, len(contours)):
        area = cv2.contourArea(contours[i])
        if area > 15:
            contours_effective.append(contours[i])

    print(len(contours_effective))
    contours = contours_effective
    img_contours = cv2.drawContours(result, contours, -1, (0, 0, 255), 2)
    filename = r'C:\Users\Iris\Desktop\OutputImg.jpg'
    cv2.imwrite(filename, img_contours)
    # cv2.imshow('img_gray with contours', img_contours)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    arr_center_x = []
    arr_center_y = []
    arr_w = []
    arr_h = []
    arr_x_min = []
    arr_y_min = []



    for contour in contours:
        area = cv2.contourArea(contour)
        x_array = contour[:, 0, 0]
        y_array = contour[:, 0, 1]
        x_mean = round(np.mean(x_array))
        y_mean = round(np.mean(y_array))
        x_min = round(np.min(x_array))
        y_min = round(np.min(y_array))
        x_max = round(np.max(x_array))
        y_max = round(np.max(y_array))

        w_contour = x_max - x_min
        h_contour = y_max - y_min

        arr_center_x.append(x_mean)
        arr_center_y.append(y_mean)
        arr_w.append(w_contour)
        arr_h.append(h_contour)
        arr_x_min.append(x_min)
        arr_y_min.append(y_min)


    # result = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)

    if len(contours) == 2:
        w_min = min(arr_w)
        idx_marker = arr_w.index(w_min)
        idx_player = 1 - idx_marker

        point_1 = [arr_center_x[idx_player], arr_center_y[idx_player]]
        point_2 = [arr_x_min[idx_marker], arr_y_min[idx_marker]]
        print(point_1)
        print(point_2)

        pixel_distance = math.sqrt(
            (point_2[0] - point_1[0]) * (point_2[0] - point_1[0]) + (point_2[1] - point_1[1]) * (
                    point_2[1] - point_1[1]))
        print('Dis_pix:', pixel_distance)
        real_d = pixel_distance * k_scale * map_scale
        real_distance_str = str(int(real_d))
        print('Real Distance:', real_distance_str)
        engine.say(real_distance_str)
        engine.runAndWait()
        engine.stop()
        return real_d

    else:
        engine.say("Error")
        engine.runAndWait()
        engine.stop()
        return 0


def auto_rotation():
    global w_scr, h_scr

    global realDistance
    global player_index
    Area = {'left': int(w_scr/2-(round(0.18*w_scr))), 'top': int(round(0.018*h_scr)),
            'width': int(round(0.36*w_scr)), 'height': int(round(0.018*h_scr))}
    with mss() as sct:
        compass_image = sct.grab(Area)
        compass_image_np = np.array(compass_image)
        hsv = cv2.cvtColor(compass_image_np, cv2.COLOR_BGR2HSV)

        # test_img = cv2.cvtColor(np.array(hsv), cv2.COLOR_HSV2BGR)
        # cv2.imshow('', test_img)
        # cv2.waitKey(0)

        if player_index == 1:  # Yellow
            lower_h = 25
            higher_h = 35
        elif player_index == 2:  # Orange
            lower_h = 8
            higher_h = 12
        elif player_index == 3:  # Blue
            lower_h = 80
            higher_h = 120
        elif player_index == 4:  # Green
            lower_h = 55
            higher_h = 65

        lower_range = np.array([lower_h, 150, 150])
        upper_range = np.array([higher_h, 255, 255])
        # Yellow 25~35; Orange 10~20
        mask = cv2.inRange(hsv, lower_range, upper_range)
        result = cv2.bitwise_and(hsv, hsv, mask=mask)
        kernel = np.ones((3, 3), np.uint8)
        result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel)
        img_bgr = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        contours, _ = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # img_contours = cv2.drawContours(compass_image_np, contours, -1, (0, 255, 0), 3)
        print('contours number', len(contours))
        if len(contours) == 1:
            contour = contours[0]
            x_array = contour[:, 0, 0]
            x_mean = round(np.mean(x_array))

            if x_mean != round(0.18*w_scr)+2:
                win32api.mouse_event(win32con.MOUSEEVENTF_MOVE, -round(0.75*(round(0.18*w_scr)-x_mean)+2), 0, 0, 0)
                print('Object Found, Moving', 'x_mean', x_mean, 'target_mean', round(0.18*w_scr)+2)
            if x_mean == round(0.18*w_scr)+2:
                if 121 < realDistance < 700:
                    if 500 < realDistance < 700:
                        realDistance_modified = realDistance - 1
                    elif 400 < realDistance < 500:
                        realDistance_modified = realDistance - 2
                    elif 300 < realDistance < 400:
                        realDistance_modified = realDistance - 3
                    elif 200 < realDistance < 300:
                        realDistance_modified = realDistance - 4
                    elif 121 < realDistance < 200:
                        realDistance_modified = realDistance - 5

                    DEG = (pi - math.asin(2 * 9.81 * realDistance_modified / (V * V))) / 2
                    Scroll_d = int(round((DEG - pi / 4) / (2 / 9 * pi) * Scroll_D, 0))
                    win32api.mouse_event(win32con.MOUSEEVENTF_WHEEL, 0, 0, Scroll_d, 0)

                time.sleep(0.1)
                win32api.mouse_event(win32con.MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0)
                win32api.mouse_event(win32con.MOUSEEVENTF_LEFTUP, 0, 0, 0, 0)
        else:
            win32api.mouse_event(win32con.MOUSEEVENTF_MOVE, -200, 0, 0, 0)
            print('Object Missing, Moving')


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
        print('zoom factor:', ZoomFactor)


LS = mouse.Listener(on_scroll=on_scroll)
LS.start()


def on_press(key):
    global ZoomFactor
    global isMeasuring
    global scale
    global realDistance
    try:
        if key.char == 'm':
            if isMeasuring:
                isMeasuring = False
            else:
                isMeasuring = True
            print('isMeasuring:', isMeasuring)

    except AttributeError:
        if key == keyboard.Key.up:
            win32api.mouse_event(win32con.MOUSEEVENTF_WHEEL, 0, 0, -Scroll_D, 0)
            print('zeroing')
        elif key == keyboard.Key.down:
            if 121 < realDistance < 700:
                if 500 < realDistance < 700:
                    realDistance_modified = realDistance - 1
                elif 400 < realDistance < 500:
                    realDistance_modified = realDistance - 2
                elif 300 < realDistance < 400:
                    realDistance_modified = realDistance - 3
                elif 200 < realDistance < 300:
                    realDistance_modified = realDistance - 4
                elif 121 < realDistance < 200:
                    realDistance_modified = realDistance - 5

                DEG = (pi - math.asin(2 * 9.81 * realDistance_modified / (V * V))) / 2
                Scroll_d = int(round((DEG - pi / 4) / (2 / 9 * pi) * Scroll_D, 0))
                win32api.mouse_event(win32con.MOUSEEVENTF_WHEEL, 0, 0, Scroll_d, 0)

        elif key == keyboard.Key.left:
            auto_rotation()

        elif key == keyboard.Key.right:
            print('Measured')
            if ZoomFactor == 1:
                scale = 100 / 18
            elif ZoomFactor == 2:
                scale = 100 / 37.9
            elif ZoomFactor == 3:
                scale = 100 / 72.1
            elif ZoomFactor == 4:
                scale = 100 / 145
            elif ZoomFactor == 5:
                scale = 100 / 289
            time.sleep(0.1)
            realDistance = auto_distance_measure(scale)
        elif key == keyboard.Key.esc:
            isMeasuring = False
            print('isMeasuring:', isMeasuring)
        elif key == keyboard.Key.tab:
            isMeasuring = False
            print('isMeasuring:', isMeasuring)


listener = keyboard.Listener(on_press=on_press)
listener.start()
engine = pyttsx3.init()
engine.setProperty('rate', 300)

# Define the GUI
sg.theme('DarkAmber')
PanelLayout = [
    [sg.Titlebar(title="PUBG-Auto Distance Measuring", background_color='chocolate4')],
    [sg.Text("Player Index"), sg.DropDown(["1", "2", "3", "4"], key='-PlayerIndex-', default_value="1"),
     sg.Text("Map Size"), sg.DropDown(["2x2", "3x3", "4x4", "6x6", "8x8"], key='-MapSize-', default_value="8x8")],
    ]
PanelWindow = sg.Window('Control Panel', PanelLayout, keep_on_top=True)

while True:
    event, values = PanelWindow.read(timeout=1)
    if event == sg.WIN_CLOSED:
        break
    player_index = int(values['-PlayerIndex-'])
    MapSize = values['-MapSize-']
    time.sleep(0.001)