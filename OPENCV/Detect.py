'''

 ▄████████    ▄████████    ▄▄▄▄███▄▄▄▄   ███    █▄     ▄████████
███    ███   ███    ███  ▄██▀▀▀███▀▀▀██▄ ███    ███   ███    ███
███    █▀    ███    ███  ███   ███   ███ ███    ███   ███    █▀
███          ███    ███  ███   ███   ███ ███    ███   ███
███        ▀███████████  ███   ███   ███ ███    ███ ▀███████████
███    █▄    ███    ███  ███   ███   ███ ███    ███          ███
███    ███   ███    ███  ███   ███   ███ ███    ███    ▄█    ███
████████▀    ███    █▀    ▀█   ███   █▀  ████████▀   ▄████████▀


╔═╗┌─┐┌┬┐┬ ┬┌─┐
║  ├─┤││││ │└─┐
╚═╝┴ ┴┴ ┴└─┘└─┘

'''

import cv2
import numpy as np
import os
import time
start = time.time()
TEMPLATE_FOLDER = "animal"
BACKGROUND_PATH = "background.png"
THRESHOLD = 0.8  # 模板匹配阈值，可调
CAMERA_INDEX = 1  # 摄像头

templates = []  # 每个元素是 (模板图像, 模板名)
for filename in os.listdir(TEMPLATE_FOLDER):
    if filename.endswith(".png"):
        path = os.path.join(TEMPLATE_FOLDER, filename)
        template = cv2.imread(path)
        template = cv2.resize(template, (0, 0), fx=0.5, fy=0.5)
        if template is not None:
            # 从文件名中提取类别，如 "monkey1.png" -> "monkey"
            animal_type = filename.split("1")[0].split("2")[0].split("3")[0].split(".")[0]
            templates.append((template, animal_type))

print(f"共加载模板 {len(templates)} 个")

# ---------------- 打开摄像头 ---------------- #
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("摄像头读取失败")
        break

    # 拷贝图像用于画框
    display_frame = frame.copy()

    detected_animals = []

    # 遍历每个模板图进行匹配
    for template, label in templates:
        temp_h, temp_w = template.shape[:2]

        # 模板匹配
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        result = cv2.matchTemplate(gray_frame, gray_template, cv2.TM_CCOEFF_NORMED)

        #result = cv2.matchTemplate(frame, template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

        # 判断是否匹配成功

        if max_val >= THRESHOLD:
            top_left = max_loc
            bottom_right = (top_left[0] + temp_w, top_left[1] + temp_h)
            center = (top_left[0] + temp_w // 2, top_left[1] + temp_h // 2)

            # 记录信息
            detected_animals.append((label, center, max_val))

            # 显示结果
            cv2.rectangle(display_frame, top_left, bottom_right, (0, 255, 0), 2)
            cv2.putText(display_frame, f"{label} ({center[0]},{center[1]})", (top_left[0], top_left[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 打印识别结果
    for label, center, score in detected_animals:
        print(f"识别到: {label} at {center}, 相似度: {score:.2f}")

    # 显示画面
    print("匹配耗时：", time.time() - start)

    cv2.imshow("Template Match Result", display_frame)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
