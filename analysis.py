import math
import json
from matplotlib.path import Path
import cv2

# json和mp4公共的文件名
# file_common_name = '2C0D5D26-42ED-45F0-84BF-C54B8D11DFF9'   # 停驶
# file_common_name = '1757C1CB-626B-4C7E-B54A-7D3A0D5F3B2C'   # 停驶
# file_common_name = '2774078E-58F1-4BBF-9A66-860385A9D93C'   # 停驶
# file_common_name = 'A333D742-A47E-45D8-A7D7-A8AAE2819CBB'   # 停驶
# file_common_name = 'A0458F86-96D5-40B2-9C90-FEE88B32F2A7'   # 停驶
# file_common_name = 'nixing1'    # 逆行
# file_common_name = '8A6569F6-74DB-4B52-B121-3269EB6B8024'   # 拥堵
# file_common_name = '25A4BBFC-A324-4EB3-8163-48FB6613268A'   # 拥堵
# file_common_name = '634EE3DE-24C1-42DF-8122-B852BBCA83FE'   # 拥堵
# file_common_name = '60-1-K96+600-336.8-341.5-1'   # 60km
# file_common_name = '60-4-K100+000-72.8-352.8-1'   # 60km
# file_common_name = '60-5-K100+600-164.6-348.8-1.1'   # 60km
# file_common_name = '60-6-K102+400-164.4-351.4-1'   # 60km
# file_common_name = '80-1-K96+600-336.8-341.5-1'   # 80km
# file_common_name = '80-4-K100+600-164.6-348.8-1.1'   # 80km
# file_common_name = '80-5-K102+400-164.4-351.4-1'   # 80km
# file_common_name = '100-1-K96+600-336.8-341.5-1'   # 100km
# file_common_name = '100-3-K100+000-72.8-352.8-1'   # 100km
# file_common_name = '100-4-K100+600-164.6-348.8-1.1'   # 100km
# file_common_name = '100-5-K102+400-164.4-351.4-1'   # 100km
# file_common_name = '丽水北站出口外广场_174_04_1_30'  # 30km
# file_common_name = '丽水北站出口外广场_166_11_1_40'  # 40km
file_common_name = '丽水北站出口外广场_174_11_1_50'  # 50km



vertices_near = []  # 近向车道的点
# vertices_near = [[0, 0], [0.1, 0], [0, 0.1], [0.1, 0.1]]  # 隧道近向车道的点
vertices_far = []   # 远向车道的点
car_lane = 4    # 车道数

# 目标和对应的label_id
# 0	car（小车）
# 1	truck（卡车）
# 2	safety_cone（安全锥）
# 3	yellow_car（施工车）
# 4	bus（客车）
# 5	tanker（油罐车）
# 6	person（行人）
# 7	reflective（反光）
# 8	motorbike（非机动车）
# 9	spill（抛撒物）
# 10 fire（火焰）
# 11 smog（烟雾）
# 12 yellow_person（施工人员）
# 13 Bumper_drum(防撞桶)
# 14 signage（标志牌）
target_label = ['car', 'truck', 'safety_cone', 'yellow_car', 'bus', 'tanker', 'person', 'reflective', 'motorbike',
                'spill', 'fire', 'smog', 'yellow_person', 'Bumper_drum', 'signage']


# P 相机的左右偏移
# T 相机的上下偏移
# Z 相机的放大倍数
# H 相机的垂直高度
# P, T, Z, H = 1, 1, 1, 5     # 隧道
P, T, Z, H = 174, 349, 1, 12
threshold_id = 0.8  # id纠偏速度阈值
# threshold_stop = 0.05  # 隧道停驶速度阈值
# threshold_slow = 1  # 隧道缓行速度阈值
threshold_stop = 2  # 停驶速度阈值
threshold_slow = 170  # 缓行速度阈值
t_interval = 0.8  # 同目标锁定最大时间间隔
t_cancel = 30  # 删除离开目标最大时间
t_cache = 30    # 缓存帧数
t_warning = 10   # 连续报警帧数


# 判断点是否在多边形内
def is_point_in_polygon(x, y, vertices):
    path = Path(vertices)
    return path.contains_point((x, y))


# 求嵌套序列中元素s的数量
# [[['缓行', 'truck_4'], ['逆行', 'yellow_car_1'], ['停驶', 'yellow_car_2'], ['', 'truck_5'], ['', 'car_11']],
#  [['缓行', 'truck_4'], ['', 'yellow_car_1'], ['', 'yellow_car_2'], ['', 'truck_5'], ['逆行', 'car_11']]]
def count_list(result):
    result_dic = {}
    for item_str in result:
        if item_str not in result_dic:
            result_dic[item_str] = 1
        else:
            result_dic[item_str] += 1
    return result_dic


def count_element(data):
    carDict = {}
    for i in range(len(data)):
        for j in range(len(data[i])):
            if data[i][j][1] in carDict:
                carDict[data[i][j][1]].extend([data[i][j][0]])
            else:
                carDict[data[i][j][1]] = [data[i][j][0]]
    for key in carDict.keys():
        carDict[key] = count_list(carDict[key])
    return carDict


# 求嵌套序列中元素s的数量
# vehicle_event: [['slow', 'yellow_car_2'], ['', 'truck_4']]
def count_slow(data, s):
    count = 0
    for i in range(len(data)):
        if data[i][0] == s:
            count += 1
        else:
            continue
    return count


def calculate_speed(t, boxes):
    """
    t: 当前画面与上一画面经过的时间
    x1min,y1min,x1max,y1max: 上一画面中车辆的 boxes
    x2min,y2min,x2max,y2max: 当前画面中车辆的 boxes
    return: 相对速度
    """
    x1min = boxes[0][0]
    y1min = boxes[0][1]
    x1max = boxes[0][2]
    y1max = boxes[0][3]
    x2min = boxes[1][0]
    y2min = boxes[1][1]
    x2max = boxes[1][2]
    y2max = boxes[1][3]
    x1mid = (x1min + x1max)/2
    y1mid = (y1min + y1max)/2
    x2mid = (x2min + x2max)/2
    y2mid = (y2min + y2max)/2
    distance = math.sqrt(pow((x1mid - x2mid), 2) + pow((y1mid - y2mid), 2))
    speed = (distance * (T-330) * H) / (Z * y2mid * t)
    # speed = (distance * T * H) / (Z * y2mid * t)
    print("distance: ",distance,"T: ",T,"H: ",H,"Z: ",Z,"y2mid: ",y2mid,"t: ",t)
    return speed


def identify_car_direction(boxes):
    """
    boxes: 该目标上一位置和当前位置
    vertices_far：远向车道的区域
    vertices_near：近向车道的区域
    return: 是否逆行（正常为正数，逆行为负数）
    """
    lane_direction = 0  # 判断车处于哪条车道（1为近向，-1为远向）
    car_direction = 0.001   # 区域外
    # 上一帧该目标的box和当前帧的该目标的box
    x1min = boxes[0][0]
    y1min = boxes[0][1]
    x1max = boxes[0][2]
    y1max = boxes[0][3]
    x2min = boxes[1][0]
    y2min = boxes[1][1]
    x2max = boxes[1][2]
    y2max = boxes[1][3]
    x1mid = (x1min + x1max) / 2
    half_y1mid = 0.75*y1max + 0.25*y1min
    x2mid = (x2min + x2max) / 2
    half_y2mid = 0.75*y2max + 0.25*y2min
    if is_point_in_polygon(x2mid, half_y2mid, vertices_near) and is_point_in_polygon(x1mid, half_y1mid, vertices_near):
        lane_direction = 1
    elif is_point_in_polygon(x2mid, half_y2mid, vertices_far) and is_point_in_polygon(x1mid, half_y1mid, vertices_far):
        lane_direction = -1
    else:
        lane_direction = 0.0000000000001    # 区域外（远距离和弯道）
    print('lane_direction: ', lane_direction, 'half_y1mid: ', half_y1mid, 'half_y2mid: ', half_y2mid)
    if lane_direction == 1 or lane_direction == -1:
        car_direction = (half_y2mid - half_y1mid) * lane_direction  # 车辆移动方向（正为正常行驶，负为逆行，0为停驶）
    return car_direction


def lock_target(target, rest_target, js_dt, num):
    """
    eg.
    第0帧，target_ids包括：1,2,3,4,5,6
    第1帧，target_ids包括：2,4,6,7
    则target：1,2,3,4,5,6,7
    rest_target：1,3,5
    new_target：7

    target: 所有到现在为止存在的目标
    rest_target: 未更新目标的下标
    js_dt: 该帧所有的目标
    num: 该目标在该帧的序号
    return：该目标原本的序号/ -1: 没有该目标
    """
    for i in range(len(rest_target)):
        if target[rest_target[i]]['label_ids'] == js_dt['label_ids'][num]:
            t = float(js_dt['time']) - float(target[rest_target[i]]['time'][-1])  # 间隔时间（s）
            box = [target[rest_target[i]]['boxes'][-1], js_dt['boxes'][num]]  # 消失前最后一帧的该目标的box和该帧该目标的box
            speed = calculate_speed(t, box)
            car_direction = identify_car_direction(box)
            if speed < threshold_id and t < t_interval and car_direction > 0:
                print(target_label[target[rest_target[i]]['label_ids']], target[rest_target[i]]['target_ids'],
                      '->', target_label[js_dt['label_ids'][num]], js_dt['target_ids'][num])
                return rest_target[i]
    return -1


def vehicle_event_detection(t, boxes):
    '''
    t: 时间间隔
    boxes:该目标上一位置和当前位置
    car_lane:车道数
    vertices_far：远向车道的区域
    vertices_near：近向车道的区域
    return: 车辆的四种检测类型：停驶、缓行、逆行/拥堵（外面判断）
    '''
    vehicle_event = ""
    speed = calculate_speed(t, boxes)
    car_direction = identify_car_direction(boxes)
    print("boxes:", boxes)
    print("speed:", speed)
    print("car_direction:", car_direction)
    if (speed < threshold_stop and abs(car_direction) > 0.1) or (speed == 0 and car_direction == 0):
        vehicle_event = "stop"
    elif speed > threshold_stop and speed < threshold_slow and car_direction > 0 and abs(car_direction) > 0.1:
        vehicle_event = "slow"
    elif car_direction < 0 and abs(car_direction) > 0.1:
        vehicle_event = "reverse"
    return vehicle_event


def main():
    available = [0, 1, 3, 4, 5]     # 用于判断车辆的label_ids
    warning = []    # 告警类型
    # 读取车辆json文件
    # path_data = 'videos/路测视频/20240723路测视频_jsons/jsons/' + file_common_name + '.json'
    # path_data = 'videos/事件类型视频/逆行/' + file_common_name + '.json'
    path_data = 'videos/路测视频/丽水北收费站0822/' + file_common_name + '.json'
    # path_data = 'videos/事件类型视频/拥堵/{' + file_common_name + '}.json'
    # path_data = 'videos/路测视频/路测0801/jsons/' + file_common_name + '.json'
    with open(path_data, 'r', encoding='UTF-8') as f_data:
        js_dt = json.load(f_data)
    target = []  # 所有目标
    target_index = []  # 主键：(target_id)*100+label_id
    # 读取车道json文件
    # path_lane = 'videos/路测视频/20240723路测视频_jsons/img/' + file_common_name + '.json'
    # path_lane = 'videos/事件类型视频/img/' + file_common_name + '.json'
    path_lane = 'videos/路测视频/丽水北收费站0822/img/' + file_common_name + '.json'
    # path_lane = 'videos/路测视频/路测0801/img/' + file_common_name + '.json'
    with open(path_lane, 'r', encoding='UTF-8') as f_lane:
        js_ln = json.load(f_lane)

    # 设置视频文件路径
    # video_path = 'videos/路测视频/20240723路测视频_jsons/videos/' + file_common_name + '.mp4'
    # video_path = 'videos/事件类型视频/逆行/' + file_common_name + '.mp4'
    # video_path = 'videos/事件类型视频/停驶/{' + file_common_name + '}.mp4'
    # 打开视频
    # cap = cv2.VideoCapture(video_path)
    # 获取视频的宽度、高度和帧率
    # width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    # height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # fps = int(cap.get(cv2.CAP_PROP_FPS))
    # video_writer = cv2.VideoWriter('stop_demo.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
    # font = cv2.FONT_HERSHEY_SIMPLEX
    # font_scale = 0.8
    # font_color = (0, 0, 255)
    # text_w = []  # 写视频文字
    # frame_w = []  # 写视频帧数
    # thickness = 2
    # count0 = 0
    # count1 = 5

    # 将远向车道区域和近向车道区域分别存储到vertices_far和vertices_near
    for i in range(len(js_ln['shapes'])):
        lane_shapes = js_ln['shapes'][i]
        if lane_shapes['label'] == '远向':
            vertices_far.extend(lane_shapes['points'])
        else:
            vertices_near.extend(lane_shapes['points'])

    for i in range(len(js_dt)):  # 第i帧
        print("frame", i)
        tmp = []    # 存储每帧的报警类型（唯一）
        vehicle_event = []  # 保存每帧每辆车的检测类型
        new_target = []
        t_now = js_dt[i]['time']
        rest_target = list(range(len(target)))
        #  匹配已存在的目标
        for j in range(len(js_dt[i]['label_ids'])):
            if js_dt[i]['label_ids'][j] in available:
                key = js_dt[i]['target_ids'][j] * 100 + js_dt[i]['label_ids'][j]    # 主键
                if key in target_index:  # 存在该目标
                    tmp_index = target_index.index(key)
                    target[tmp_index]['time'].extend([js_dt[i]['time']])
                    target[tmp_index]['boxes'].extend([js_dt[i]['boxes'][j]])
                    rest_target.remove(tmp_index)
                else:   # 不存在该目标
                    new_target.extend([j])
        #  匹配新的目标
        for j in range(len(new_target)):
            target_ans = lock_target(target, rest_target, js_dt[i], new_target[j])
            key = js_dt[i]['target_ids'][new_target[j]] * 100 + js_dt[i]['label_ids'][new_target[j]]
            if target_ans == -1:  # 新目标
                target_index.extend([key])   # 加下标
                target.extend([{'label_ids': js_dt[i]['label_ids'][new_target[j]],  # 加目标
                                'target_ids': js_dt[i]['target_ids'][new_target[j]],
                                'boxes': [js_dt[i]['boxes'][new_target[j]]], 'time': [js_dt[i]['time']]}])
            else:  # 锁定已存在的目标
                target[target_ans]['target_ids'] = js_dt[i]['target_ids'][new_target[j]]
                target_index[target_ans] = key
                target[target_ans]['time'].extend([js_dt[i]['time']])
                target[target_ans]['boxes'].extend([js_dt[i]['boxes'][new_target[j]]])
                rest_target.remove(target_ans)

        for j in range(len(target)):
            if target[j]['time'][-1] == t_now and len(target[j]['time']) > 1:
                t = float(target[j]['time'][-1]) - float(target[j]['time'][-2])  # 间隔时间（s）
                boxes = [target[j]['boxes'][-2], target[j]['boxes'][-1]]  # 消失前最后一帧的该目标的box和该帧该目标的box
                key = target_label[target[j]['label_ids']] + '_' + str(target[j]['target_ids'])
                print('key:', key)
                vehicle_event.append([vehicle_event_detection(t, boxes), key])

        print("vehicle_event:", vehicle_event)

        for j in range(len(rest_target) - 1, -1, -1):  # 清空缓存
            if t_cancel < float(js_dt[i]['time']) - float(target[rest_target[j]]['time'][-1]):
                # 更新时间大于t_cancel的，删除
                del target[rest_target[j]]
                del target_index[rest_target[j]]
        print('slow_count:', count_slow(vehicle_event, 'slow'))
        print('stop_count:', count_slow(vehicle_event, 'stop'))
        if (count_slow(vehicle_event, 'slow') + count_slow(vehicle_event, 'stop')) > car_lane * 2:
            vehicle_event.append(["congestion", ''])
        if len(warning) < t_cache:
            warning.append(vehicle_event)
        else:
            warning = []
            warning.append(vehicle_event)
        # print("warning:", warning)
        carDict = count_element(warning)
        for key in carDict:
            for item in carDict[key]:
                if carDict[key][item] == t_warning and item != '':
                    print(key, item, '*******************')
                    # frame_w.append(i)
                    # text_w.append(key + ' ' + item)
                    warning = []
                    print(warning)
                    continue
    # 写入视频
    # # 读取视频帧
    # while True:
    #     ret, frame = cap.read()
    #     if not ret:
    #         break
    #     current_frame_number = cap.get(1)
    #     print(current_frame_number)
    #     for i in range(len(frame_w)):
    #         if current_frame_number >= frame_w[i] and current_frame_number < (frame_w[i] + count1):
    #             cv2.putText(frame, text_w[i], (50, 50), font, font_scale, font_color, thickness)
    #             count0 += 1
    #         else:
    #             count0 = 0
    #         # video_writer.write(frame)
    #
    # # 释放视频对象和关闭窗口
    # cap.release()
    # video_writer.release()
    # cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
