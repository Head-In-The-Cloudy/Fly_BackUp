#官方例程
#
import cv2
import numpy as np

threshold = 0.8

img = cv2.imread('mario.jpg')
img1=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
template = cv2.imread('mario_coin.jpg',0)
h, w = template.shape[:2]
res = cv2.matchTemplate(img1, template, cv2.TM_CCOEFF_NORMED)
loc = np.where( res >= threshold)
for pt in zip(*loc[::-1]):
    cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h),(0,255,0), 1)

cv2.imshow("Result", img)
cv2.imshow("mario_coin",template)
cv2.waitKey(0)

#自己多模板匹配
# _*_coding:utf-8_*_
# 作者：   Java Punk
# 时间：   2022-10-09 14:49:45

import cv2 as cv2
import numpy as np


# 多个模板匹配
def more_match(image, templ):
    img = cv2.imread(image)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    template = cv2.imread(templ, 0)
    h, w = template.shape[:2]

    res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
    # 取匹配程度大于%90的坐标
    threshold = 0.9
    # np.where返回的坐标值(x,y)是(h,w)，注意h,w的顺序
    loc = np.where(res >= threshold)
    for pt in zip(*loc[::-1]):
        bottom_right = (pt[0] + w, pt[1] + h)
        cv2.rectangle(img, pt, bottom_right, (255, 0, 0), 1)
        print(pt, bottom_right)
    cv2.imshow('img_rgb', img)
    cv2.waitKey(0)
    pass


if __name__ == '__main__':
    print("———————————————————— start ————————————————————\n")
    # 图片路径自己设置，下面是我本地的路径，记得替换！！！
    more_match('../img/test/zhipai_03.jpg', '../img/test/zhipai_04.jpg')
    print("———————————————————— end ————————————————————\n")
