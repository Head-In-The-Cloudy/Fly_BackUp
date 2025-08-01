from maix import camera, display, image, nn, app
model_width = 224  
model_height = 224
detector = nn.YOLOv5(model="/root/models/maixhub/229475/model_229475.mud", dual_buff = True)

#cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
cam = camera.Camera(640, 480,  detector.input_format())

disp = display.Display()
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
ROI_WIDTH =  380 # New_ROI宽度 原640
ROI_HEIGHT = 340  # New_ROI高度 原480
roi_x = (IMAGE_WIDTH - ROI_WIDTH) // 2  # 水平居中
roi_y = (IMAGE_HEIGHT - ROI_HEIGHT) // 2  # 垂直居中

while not app.need_exit():
    # 读取完整分辨率的图像
    img = cam.read()
    
    # 创建ROI图像（不改变原始图像）
    roi_img = img.crop(roi_x, roi_y, ROI_WIDTH, ROI_HEIGHT)
    
    # 将ROI图像缩放到模型输入尺寸
    model_img = roi_img.resize(model_width, model_height)
    
    # 在模型尺寸图像上进行检测
    objs = detector.detect(model_img, conf_th=0.5, iou_th=0.45)
    
    # 在原图上绘制ROI边界
    img.draw_rect(roi_x, roi_y, ROI_WIDTH, ROI_HEIGHT, color=image.COLOR_BLUE)
    
    # 处理检测结果
    for obj in objs:
        # 将模型坐标转换回ROI坐标
        roi_x1 = int(obj.x * ROI_WIDTH / model_width)
        roi_y1 = int(obj.y * ROI_HEIGHT / model_height)
        roi_x2 = int((obj.x + obj.w) * ROI_WIDTH / model_width)
        roi_y2 = int((obj.y + obj.h) * ROI_HEIGHT / model_height)
        
        # 将ROI坐标转换回原图坐标
        abs_x1 = roi_x + roi_x1
        abs_y1 = roi_y + roi_y1
        abs_x2 = roi_x + roi_x2
        abs_y2 = roi_y + roi_y2
        w = abs_x2 - abs_x1
        h = abs_y2 - abs_y1
        
        # 在原图上绘制检测框
        img.draw_rect(abs_x1, abs_y1, w, h, color=image.COLOR_RED)
        msg = f'{detector.labels[obj.class_id]}: {obj.score:.2f}'
        img.draw_string(abs_x1, abs_y1, msg, color=image.COLOR_RED)
    
    # 显示原图（带ROI和检测框）
    disp.show(img)