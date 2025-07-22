import gc
import os
import utime

import aicube
import image
import nncase_runtime as nn
import ujson
import ulab.numpy as np
from libs.PipeLine import ScopedTiming
from libs.Utils import *
from media.display import *
from media.media import *
from media.sensor import *

from machine import UART
from machine import FPIOA

display_mode = "lcd"
if display_mode == "lcd":
    DISPLAY_WIDTH = ALIGN_UP(800, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(640, 16)
OUT_RGB888P_HEIGH = 360

root_path = "/sdcard/projects/送药小车V2/mp_deployment_source/"
config_path = root_path + "deploy_config.json"
deploy_conf = {}
debug_mode = 0

#串口配置
fpioa = FPIOA()

# UART3
fpioa.set_function(50,fpioa.UART3_TXD)
fpioa.set_function(51, fpioa.UART3_RXD)
# 串口配置
uart = UART(UART.UART3, 115200)



def two_side_pad_param(input_size, output_size):
    ratio_w = output_size[0] / input_size[0]  # 宽度缩放比例
    ratio_h = output_size[1] / input_size[1]  # 高度缩放比例
    ratio = min(ratio_w, ratio_h)  # 取较小的缩放比例
    new_w = int(ratio * input_size[0])  # 新宽度
    new_h = int(ratio * input_size[1])  # 新高度
    dw = (output_size[0] - new_w) / 2  # 宽度差
    dh = (output_size[1] - new_h) / 2  # 高度差
    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw - 0.1))
    return top, bottom, left, right, ratio


def read_deploy_config(config_path):
    # 打开JSON文件以进行读取deploy_config
    with open(config_path, "r") as json_file:
        try:
            # 从文件中加载JSON数据
            config = ujson.load(json_file)
        except ValueError as e:
            print("JSON 解析错误:", e)
    return config

def find_top_two_modes(arr):
    if not arr:
        return None, None

    # 使用字典统计每个数字出现的频率
    freq = {}
    for num in arr:
        freq[num] = freq.get(num, 0) + 1

    # 如果没有重复值
    if len(freq) == 1:
        return list(freq.keys())[0], None

    # 获取频率最高的两个数字
    sorted_items = sorted(freq.items(), key=lambda x: (-x[1], -x[0]))
    return sorted_items[0][0], sorted_items[1][0]

def find_mode(arr):
    if not arr:
        return None

    # 使用字典统计频率
    freq = {}
    for num in arr:
        freq[num] = freq.get(num, 0) + 1

    # 找到出现次数最多的元素
    max_count = 0
    mode = None
    for num, count in freq.items():
        if count > max_count:
            max_count = count
            mode = num

    return mode

def detection():
    global uart
    print("det_infer start")
    # 使用json读取内容初始化部署变量
    deploy_conf = read_deploy_config(config_path)
    kmodel_name = deploy_conf["kmodel_path"]
    labels = deploy_conf["categories"]
    confidence_threshold = deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size = deploy_conf["img_size"]
    num_classes = deploy_conf["num_classes"]
    color_four = get_colors(num_classes)
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]
    if model_type == "AnchorBaseDet":
        anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]
    kmodel_frame_size = img_size
    frame_size = [OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGH]
    strides = [8, 16, 32]

    # 计算padding值
    top, bottom, left, right, ratio = two_side_pad_param(frame_size, kmodel_frame_size)

    # 初始化kpu
    kpu = nn.kpu()
    kpu.load_kmodel(root_path + kmodel_name)
    # 初始化ai2d
    ai2d = nn.ai2d()
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
    ai2d_builder = ai2d.build(
        [1, 3, OUT_RGB888P_HEIGH, OUT_RGB888P_WIDTH], [1, 3, kmodel_frame_size[1], kmodel_frame_size[0]]
    )
    # 初始化并配置sensor
    sensor = Sensor()
    sensor.reset()
    # 设置镜像
    sensor.set_hmirror(False)
    # 设置翻转
    sensor.set_vflip(False)
    # 通道0直接给到显示VO，格式为YUV420
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    # 通道2给到AI做算法处理，格式为RGB888
    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)
    # 绑定通道0的输出到vo
    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
    if display_mode == "lcd":
        # 设置为ST7701显示，默认800x480
        Display.init(Display.ST7701, to_ide=True)
    else:
        # 设置为LT9611显示，默认1920x1080
        Display.init(Display.LT9611, to_ide=True)
    # 创建OSD图像
    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    # media初始化
    MediaManager.init()
    # 启动sensor
    sensor.run()
    rgb888p_img = None
    ai2d_input_tensor = None
    data = np.ones((1, 3, kmodel_frame_size[1], kmodel_frame_size[0]), dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)

    #时间控制变量
    last_line_send_time=utime.ticks_ms()#记录上次巡线信息发送时间
    line_send_interval=100 #发送间隔100ms
    #红竖直线中心横坐标
    line_cx=0

    #竖直线检验标志
    line_detected=False

    #使能交叉路口检测
    enable_cross_detected=False
    #交叉路口验证计时器
    cross_confirm_cnt=0
    #确认交叉路口检测标志
    cross_detected=False

    #使能药房/病房检测
    enable_block_detected=False
    #药房/病房验证计时器
    block_confirm_cnt=0
    #确认药房/病房检测标志
    block_detected=False

    #使能检测数字标志
    enable_number_detected=False
    #确认识别数字标志
    number_detected=False
    #检测的数字数量
    numbers_required=4

    #数字检测验证方案
    numbers_confirm_list=[]#储存识别到的数字序列,进行数字验证
    left_numbers_list=None
    right_numbers_list=None
    numbers_confirm_cnt=0 #数字验证阈值
   #当前发送串口命令时间
    current_time=0

    uart.write("Hello From K230!\r\n")#发送串口信息

    left_numbers_list=[]
    right_numbers_list=[]
    mode1=0
    mode2=0
    mode=0
    OneFlag=False
    TwoFlag=False
    FourFlag=False
    Num=0
    Num1=0
    Num2=0
    Num11=0
    Num21=0
    Num31=0
    Num41=0

    while True:

        current_time=utime.ticks_ms()#获取当前时间

        # 串口接收与处理信息 - 使用非阻塞方式
        if uart.any():  # 检查是否有数据可读
            message = uart.read()  # 读取所有可用数据
            try:
                 # 尝试解码为字符串
                 message_str = message.decode('utf-8').strip()
                 print(f"Received raw: {message}")
                 print(f"Received decoded: {message_str}")

                 # 命令处理
                 if "detect one number" in message_str:
                     enable_number_detected = True
                     numbers_required = 1
                     print("开始识别一个数字!")

                 elif "detect two numbers" in message_str:
                     enable_number_detected = True
                     numbers_required = 2
                     print("开始识别两个数字")

                 elif "detect four numbers" in message_str:
                     enable_number_detected = True
                     numbers_required = 4
                     print("开始识别四个数字")

                 elif "enable cross detected" in message_str:
                     enable_cross_detected = True

                 elif "enable block detected" in message_str:
                      enable_block_detected = True

                 elif "Hello From STM32!" in message_str:
                     print("收到STM32的问候！")



            except UnicodeError:
               print("解码错误，接收的数据:", message)
            except Exception as e:
              print("处理串口数据时出错:", e)




        with ScopedTiming("total", debug_mode > 0):
            rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
            if rgb888p_img.format() == image.RGBP888:
                ai2d_input = rgb888p_img.to_numpy_ref()
                ai2d_input_tensor = nn.from_numpy(ai2d_input)
                # 使用ai2d进行预处理
                ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)
                # 设置模型输入
                kpu.set_input_tensor(0, ai2d_output_tensor)
                # 模型推理
                kpu.run()
                # 获取模型输出
                results = []
                for i in range(kpu.outputs_size()):
                    out_data = kpu.get_output_tensor(i)
                    result = out_data.to_numpy()
                    result = result.reshape((result.shape[0] * result.shape[1] * result.shape[2] * result.shape[3]))
                    del out_data
                    results.append(result)
                # 使用aicube模块封装的接口进行后处理
                det_boxes = aicube.anchorbasedet_post_process(
                    results[0],
                    results[1],
                    results[2],
                    kmodel_frame_size,
                    frame_size,
                    strides,
                    num_classes,
                    confidence_threshold,
                    nms_threshold,
                    anchors,
                    nms_option,
                )
                # 绘制结果
                osd_img.clear()

                number_list=[] #当前帧识别到的数字
                cx_list=[] #当前识别的数字中心列表



                if det_boxes:
                    for det_boxe in det_boxes:
                        x1, y1, x2, y2 = det_boxe[2], det_boxe[3], det_boxe[4], det_boxe[5]
                        x = int(x1 * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                        y = int(y1 * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                        w = int((x2 - x1) * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                        h = int((y2 - y1) * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                        osd_img.draw_rectangle(x, y, w, h, color=color_four[det_boxe[0]][1:])
                        text = labels[det_boxe[0]] + " " + str(round(det_boxe[1], 2))
                        osd_img.draw_string_advanced(x, y - 40, 32, text, color=color_four[det_boxe[0]][1:])


                        #获取当前识别的标签
                        label=labels[det_boxe[0]]

                        #提取巡线信息
                        if label=="竖直线":
                            line_detected=True
                            cx=(x1+x2)//2

                        #提取交叉路口信息
                        if enable_cross_detected:
                            if label=="交叉路口":
                                cross_confirm_cnt+=1
                                if cross_confirm_cnt>=5:#5帧检验
                                    cross_detected=1
                                    cross_confirm_cnt=0

                        #提取黑块信息
                        if enable_block_detected:
                            if label=="黑块":
                                block_confirm_cnt+=1
                                if block_confirm_cnt>=10:#5帧检验
                                    block_detected=1
                                    block_confirm_cnt=0

                        #提取数字信息
                        if enable_number_detected:
                                number=det_boxe[0]+1
                                probability=round(det_boxe[1], 2)
                                if 1<=number<=8:
                                  number_list.append(number)
                                  number_cx=(x1+x2)//2
                                  cx_list.append(number_cx)
                                  if(number_cx<=320 and probability>0.4):
                                      left_numbers_list.append(number)
                                  elif(number_cx>320 and probability>0.4):
                                      right_numbers_list.append(number)


                #鲁棒数字识别

                #if number_list != None:
                    #number_list按照cx_list升序排序
                    '''
                    indices = sorted(range(len(cx_list)), key=lambda i: cx_list[i])
                    number_list = [number_list[i] for i in indices]
                    '''

                    '''
                    if number_list != None:
                        numbers_confirm_list.append(number_list)
                        length=len(numbers_confirm_list)
                        if length>1:
                            if(numbers_confirm_list[length-1]==numbers_confirm_list[length-2]):
                                numbers_confirm_cnt+=1
                                if(numbers_confirm_cnt>=20):#20帧验证
                                    number_detected=True #连续20帧检测到数字相同，确认识别到的数字信息
                                    numbers_confirm_list=[]
                                    numbers_confirm_cnt=0
                    '''

                if left_numbers_list!=None and right_numbers_list!=None:
                    numbers_confirm_cnt+=1
                    if(numbers_confirm_cnt>=20):#连续20帧验证
                        number_detected=True
                        numbers_confirm_cnt=0




                #控制巡线信息发送周期:100ms
                if line_detected:

                   # print(f"time:{current_time},{last_line_send_time}")
                    if utime.ticks_diff(current_time,last_line_send_time)>=line_send_interval:
                        uart.write(f"cx:{cx}\r\n") #给单片机发送竖直线信息
                        print(f"识别到竖直线，中心位置:{cx}")
                        last_line_send_time=current_time#更新上一次发送串口信息时间
                        line_detected=False

                #发送识别到的数字信息
                '''
                if number_detected:
                    if numbers_required==1 and len(number_list)==1:
                         uart.write(f"detect one number: {number_list[0]}\r\n")
                         print(f"识别到一个数字:{number_list[0]}")
                    elif numbers_required==2 and len(number_list)==2:
                         uart.write(f"detect two numbers: {number_list[0]} {number_list[1]}\r\n")
                         print(f"识别到两个数字: {number_list[0]} {number_list[1]}")
                    elif numbers_required==4 and len(number_list)==4:
                         print(f"识别到四个数字: {number_list[0]} {number_list[1]} {number_list[2]} {number_list[3]}\r\n")
                         uart.write(f"detect four numbers: {number_list[0]} {number_list[1]} {number_list[2]} {number_list[3]}")
                    number_detected=False
                    numbers_required=0
                '''
                if number_detected:
                    if numbers_required==1:
                        mode=find_mode(number_list)
                        if(mode!=None):
                          uart.write(f"detect one number: {mode}\r\n")
                          OneFlag=True
                          Num=mode
                          #osd_img.draw_string_advanced(320, 180, 60, f" {mode}")
                          print(f"识别到一个数字:{mode}")
                    elif numbers_required==2:
                        mode1=find_mode(left_numbers_list)
                        mode2=find_mode(right_numbers_list)
                        if(mode1!=None and mode2!=None):
                            uart.write(f"detect two numbers: {mode1} {mode2}\r\n")
                            #osd_img.draw_string_advanced(320, 240, 60, f"{mode1} {mode2}")
                            TwoFlag=True
                            Num1=mode1
                            Num2=mode2
                            print(f"识别到两个数字: {mode1} {mode2}")
                    elif numbers_required==4:
                        mode1,mode2=find_top_two_modes(left_numbers_list)
                        mode3,mode4=find_top_two_modes(right_numbers_list)
                        if(mode1!=None and mode2!=None and mode3!=None and mode4!=None):
                            uart.write(f"detect four numbers: {mode1} {mode2} {mode3} {mode4}\r\n")
                            FourFlag=True
                            Num11=mode1
                            Num12=mode2
                            Num13=mode3
                            Num14=mode4
                            print(f"识别到四个数字: {mode1} {mode2} {mode3} {mode4}")
                    number_detected=False
                    numbers_required=0
                    enable_number_detected=False
                    left_numbers_list=[]
                    right_numbers_list=[]
                    number_list=[]

                #发送识别到交叉路口信息
                if cross_detected:
                    uart.write(f"detect cross\r\n")
                    print(f"识别到交叉路口")
                    cross_detected=False
                    enable_cross_detected=False

                #发送识别到药房/病房信息
                if block_detected:
                    uart.write(f"detect block\r\n")
                    print(f"识别到药房/病房")
                    block_detected=False
                    enable_block_detected=False
                #if TwoFlag==True:
                    #osd_img.draw_string_advanced(320, 240, 60, f"{Num1} {Num2}")
                #if OneFlag==True:
                    #osd_img.draw_string_advanced(320, 120, 60, f"{Num}")
                #if FourFlag==True:
                    #osd_img.draw_string_advanced(320,360,60,f"{Num11},{Num12},{Num13},{Num14}")
                Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)

                gc.collect()
            rgb888p_img = None
    del ai2d_input_tensor
    del ai2d_output_tensor
    # 停止摄像头输出
    sensor.stop()
    # 去初始化显示设备
    Display.deinit()
    # 释放媒体缓冲区
    MediaManager.deinit()
    gc.collect()
    time.sleep(1)
    nn.shrink_memory_pool()
    print("det_infer end")
    return 0


if __name__ == "__main__":
    detection()
