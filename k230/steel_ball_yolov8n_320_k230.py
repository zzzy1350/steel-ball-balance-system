"""K230钢球识别、卡尔曼滤波和STM32通信主程序。

本程序使用320x320的摄像头画面和320x320的YOLOv8模型识别钢球，
然后从检测框中心得到钢球坐标。为了降低目标框抖动对舵机控制的影响，
程序分别在x、y方向运行“位置+速度”恒速度卡尔曼滤波器。

K230通过UART1与STM32F103C8T6双向通信：
    GPIO3：UART1_TX，连接STM32 PA10/USART1_RX；
    GPIO4：UART1_RX，连接STM32 PA9/USART1_TX；
    波特率：115200，8数据位，无校验，1停止位。

注意：K230、STM32和舵机电源必须共地。舵机必须使用独立5V电源，
不能由K230或STM32的3.3V引脚直接供电。
"""

from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.PipeLine import ScopedTiming
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA
import nncase_runtime as nn
import ulab.numpy as np
import aidemo
import time
import gc
import sys


# =============================================================================
# 模型、摄像头和显示参数
# =============================================================================

KMODEL_PATH = "/sdcard/models/steel_ball_yolov8n_320_new.kmodel"
LABELS = ["steel_ball"]
MODEL_INPUT_SIZE = [320, 320]
EXPECTED_OUTPUT_POINTS = 2100

# 摄像头AI通道、显示通道和模型输入都使用320x320，坐标可以直接一一对应。
AI_FRAME_SIZE = [320, 320]
DISPLAY_SIZE = [320, 320]

CONF_THRESH = 0.35
NMS_THRESH = 0.45  #两个检测框重叠到什么程度时，其中一个会被删除
MAX_BOXES_NUM = 3

ENABLE_DISPLAY = True
DISPLAY_EVERY_N_FRAMES = 1
PRINT_EVERY_N_FRAMES = 30
GC_EVERY_N_FRAMES = 30
DRAW_BOX_SCORE = True
PRINT_BOX_DETAILS = False

SUMMARY_FONT_SIZE = 16
BOX_FONT_SIZE = 14
SUMMARY_BAR_HEIGHT = 20
DEBUG_MODE = 0


# =============================================================================
# 单球关联和卡尔曼滤波参数
# =============================================================================

# 检测框中心与卡尔曼预测点的最大允许距离。超过该距离的检测框不会接管当前轨迹。
ASSOCIATION_GATE_PIXELS = 60.0

# 测量方差。标准差取4像素，因此方差为4平方，即16。
KALMAN_MEASUREMENT_VARIANCE = 9.0

# 过程模型使用的加速度噪声标准差，单位为像素/秒平方。
KALMAN_ACCELERATION_STD = 120.0

# 第一次检测到钢球时使用的协方差初值。
KALMAN_INITIAL_POSITION_VARIANCE = 100.0
KALMAN_INITIAL_VELOCITY_VARIANCE = 10000.0

# 帧间隔过小或过大都会使卡尔曼模型数值不稳定，因此进行合理限幅。
KALMAN_MIN_DT_SECONDS = 0.01
KALMAN_MAX_DT_SECONDS = 0.20

# 漏检不超过3帧时允许发送预测坐标；连续漏检10帧后彻底重置轨迹。
MAX_PREDICT_FRAMES = 3
RESET_AFTER_MISSED_FRAMES = 10

TRACK_MEASURED = 1
TRACK_PREDICTED = 2
TRACK_LOST = 3
TRACK_RESET = 4


# =============================================================================
# UART和固定14字节协议参数
# =============================================================================

UART_ID = UART.UART1
UART_TX_PIN = 3
UART_RX_PIN = 4
UART_TX_FUNCTION = FPIOA.UART1_TXD
UART_RX_FUNCTION = FPIOA.UART1_RXD
UART_BAUDRATE = 115200

PROTOCOL_FRAME_SIZE = 14
PROTOCOL_SOF1 = 0xAA
PROTOCOL_SOF2 = 0x55
PROTOCOL_VERSION = 0x01

MSG_BALL_STATE = 0x01
MSG_SET_TARGET = 0x10
MSG_TARGET_ACK = 0x11

FLAG_POSITION_VALID = 1 << 0
FLAG_CURRENT_MEASURED = 1 << 1
FLAG_CURRENT_PREDICTED = 1 << 2
FLAG_KALMAN_INITIALIZED = 1 << 3

INVALID_COORDINATE = 0xFFFF
DEFAULT_TARGET_X = 160
DEFAULT_TARGET_Y = 160


def clamp(value, minimum, maximum):
    """把数值限制在指定闭区间内。

    参数：value为待限制数值，minimum和maximum分别为允许下限和上限。
    返回：位于[minimum, maximum]内的数值。调用者应保证下限不大于上限。
    """
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def get_monotonic_ms():
    """返回单调递增的毫秒时间。

    新版CanMV固件通常提供ticks_ms；若固件没有该接口，则退回到time.time。
    time.time在部分固件上的分辨率较低，因此部署时优先使用带ticks_ms的固件。
    """
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    if hasattr(time, "ticks_us"):
        return time.ticks_us() // 1000
    if hasattr(time, "time_ns"):
        return time.time_ns() // 1000000
    return int(time.time() * 1000)


def get_elapsed_ms(now_ms, previous_ms):
    """计算两个毫秒时刻之间的间隔。

    参数：now_ms为当前时刻，previous_ms为上一次时刻，单位均为毫秒。
    返回：经过的毫秒数。固件提供ticks_diff时可正确处理计数回绕。
    """
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now_ms, previous_ms)
    return now_ms - previous_ms


def crc16_modbus(data, start, length):
    """计算CRC16/MODBUS。

    参数：
        data：包含待校验字节的bytes或bytearray；
        start：第一个参与CRC计算的下标；
        length：参与计算的字节数。
    返回：
        16位CRC值，协议发送时低字节在前。
    """
    crc = 0xFFFF
    end = start + length
    for index in range(start, end):
        crc ^= data[index]
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def put_u16_le(buffer, index, value):
    """向bytearray指定位置写入一个小端16位无符号整数。"""
    buffer[index] = value & 0xFF
    buffer[index + 1] = (value >> 8) & 0xFF


def get_u16_le(buffer, index):
    """从bytes或bytearray指定位置读取一个小端16位无符号整数。"""
    return buffer[index] | (buffer[index + 1] << 8)


class KalmanAxis:
    """单轴恒速度卡尔曼滤波器。

    状态向量只有两个量：位置和速度。为了减少K230上的内存分配和矩阵运算，
    这里直接展开2x2矩阵公式，而不在每一帧创建临时矩阵。
    """

    def __init__(self):
        self.initialized = False
        self.position = 0.0
        self.velocity = 0.0
        self.p00 = KALMAN_INITIAL_POSITION_VARIANCE
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = KALMAN_INITIAL_VELOCITY_VARIANCE

    def reset(self):
        """清除状态和协方差，等待下一次真实测量重新初始化。"""
        self.initialized = False
        self.position = 0.0
        self.velocity = 0.0
        self.p00 = KALMAN_INITIAL_POSITION_VARIANCE
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = KALMAN_INITIAL_VELOCITY_VARIANCE

    def initialize(self, measurement):
        """使用第一次真实测量初始化位置，初始速度设为0。"""
        self.initialized = True
        self.position = float(measurement)
        self.velocity = 0.0
        self.p00 = KALMAN_INITIAL_POSITION_VARIANCE
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = KALMAN_INITIAL_VELOCITY_VARIANCE

    def predict(self, dt):
        """按照恒速度模型进行一步预测。"""
        if not self.initialized:
            return

        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        acceleration_variance = KALMAN_ACCELERATION_STD * KALMAN_ACCELERATION_STD

        old_p00 = self.p00
        old_p01 = self.p01
        old_p10 = self.p10
        old_p11 = self.p11

        self.position += self.velocity * dt

        # Q矩阵来自白噪声加速度模型：
        # q * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]。
        self.p00 = (
            old_p00
            + dt * (old_p01 + old_p10)
            + dt2 * old_p11
            + 0.25 * dt4 * acceleration_variance
        )
        self.p01 = old_p01 + dt * old_p11 + 0.5 * dt3 * acceleration_variance
        self.p10 = old_p10 + dt * old_p11 + 0.5 * dt3 * acceleration_variance
        self.p11 = old_p11 + dt2 * acceleration_variance

    def update(self, measurement):
        """使用新的位置测量修正预测状态。"""
        if not self.initialized:
            self.initialize(measurement)
            return

        innovation = float(measurement) - self.position
        innovation_variance = self.p00 + KALMAN_MEASUREMENT_VARIANCE
        if innovation_variance <= 0.000001:
            return

        gain_position = self.p00 / innovation_variance
        gain_velocity = self.p10 / innovation_variance

        old_p00 = self.p00
        old_p01 = self.p01
        old_p10 = self.p10
        old_p11 = self.p11

        self.position += gain_position * innovation
        self.velocity += gain_velocity * innovation

        self.p00 = (1.0 - gain_position) * old_p00
        self.p01 = (1.0 - gain_position) * old_p01
        self.p10 = old_p10 - gain_velocity * old_p00
        self.p11 = old_p11 - gain_velocity * old_p01

        # 浮点舍入会让P01和P10产生极小差异，主动对称化可减小长期误差。
        symmetric = 0.5 * (self.p01 + self.p10)
        self.p01 = symmetric
        self.p10 = symmetric


class BallTracker:
    """二维钢球轨迹管理器，负责预测、检测框关联、漏检计数和重置。"""

    def __init__(self):
        self.x_filter = KalmanAxis()
        self.y_filter = KalmanAxis()
        self.missed_frames = 0
        self.last_confidence = 0.0
        self.state = TRACK_RESET

    def is_initialized(self):
        """返回x、y两个滤波器是否都已经由真实测量初始化。"""
        return self.x_filter.initialized and self.y_filter.initialized

    def reset(self):
        """清除二维滤波状态、漏检计数和置信度，等待下一次真实检测。"""
        self.x_filter.reset()
        self.y_filter.reset()
        self.missed_frames = 0
        self.last_confidence = 0.0
        self.state = TRACK_RESET

    def predict(self, dt):
        """使用实际帧间隔进行二维预测；尚未初始化时不执行任何操作。

        参数：dt为本帧时间间隔，单位秒，调用前已经限制到0.01～0.20秒。
        """
        if self.is_initialized():
            self.x_filter.predict(dt)
            self.y_filter.predict(dt)

    def select_detection(self, detections):
        """从YOLO结果中选择当前要跟踪的单个钢球框。"""
        if not detections:
            return None

        if not self.is_initialized():
            best = detections[0]
            for detection in detections[1:]:
                if detection[4] > best[4]:
                    best = detection
            return best

        predicted_x = self.x_filter.position
        predicted_y = self.y_filter.position
        gate_squared = ASSOCIATION_GATE_PIXELS * ASSOCIATION_GATE_PIXELS
        best = None
        best_distance_squared = gate_squared

        for detection in detections:
            center_x = 0.5 * (detection[0] + detection[2])
            center_y = 0.5 * (detection[1] + detection[3])
            dx = center_x - predicted_x
            dy = center_y - predicted_y
            distance_squared = dx * dx + dy * dy
            if distance_squared <= best_distance_squared:
                best_distance_squared = distance_squared
                best = detection
        return best

    def update_with_detection(self, detection):
        """使用选中检测框的中心点更新二维滤波器。"""
        raw_x = 0.5 * (detection[0] + detection[2])
        raw_y = 0.5 * (detection[1] + detection[3])
        self.x_filter.update(raw_x)
        self.y_filter.update(raw_y)
        self.missed_frames = 0
        self.last_confidence = detection[4]
        self.state = TRACK_MEASURED
        return raw_x, raw_y

    def update_without_detection(self):
        """处理本帧漏检，并根据连续漏检数量决定是否仍允许预测。"""
        if not self.is_initialized():
            self.state = TRACK_LOST
            return

        self.missed_frames += 1
        if self.missed_frames <= MAX_PREDICT_FRAMES:
            self.state = TRACK_PREDICTED
        else:
            self.state = TRACK_LOST

        if self.missed_frames >= RESET_AFTER_MISSED_FRAMES:
            self.reset()

    def position_is_valid(self):
        """返回当前坐标能否发送给STM32参与控制。

        只有滤波器已初始化且状态为真实测量或前三帧短时预测时才返回True。
        """
        return self.is_initialized() and self.state in (TRACK_MEASURED, TRACK_PREDICTED)

    def get_position(self):
        """返回当前卡尔曼位置元组(x, y)，单位像素。

        调用前应先检查position_is_valid；丢球状态下内部预测值不允许用于控制。
        """
        return self.x_filter.position, self.y_filter.position


class BalanceProtocol:
    """K230端固定14字节协议收发器。

    接收函数不会阻塞等待数据。它会保留未组成完整帧的字节，并能从串口噪声、
    半包、粘包和CRC错误后重新找到下一帧帧头。
    """

    def __init__(self, uart):
        self.uart = uart
        self.rx_buffer = bytearray()
        self.tx_sequence = 0
        self.valid_frames = 0
        self.crc_errors = 0

    def build_frame(self, message_type, flags, x, y, auxiliary):
        """构造一帧固定14字节二进制数据并自动递增发送序号。

        参数分别为消息类型、状态标志、x、y和16位辅助数据。
        返回：包含帧头、小端字段和CRC16/MODBUS的bytearray。
        所有字段在写入前限制到协议规定的无符号位宽。
        """
        frame = bytearray(PROTOCOL_FRAME_SIZE)
        frame[0] = PROTOCOL_SOF1
        frame[1] = PROTOCOL_SOF2
        frame[2] = PROTOCOL_VERSION
        frame[3] = message_type & 0xFF
        frame[4] = self.tx_sequence
        frame[5] = flags & 0xFF
        put_u16_le(frame, 6, x & 0xFFFF)
        put_u16_le(frame, 8, y & 0xFFFF)
        put_u16_le(frame, 10, auxiliary & 0xFFFF)
        crc = crc16_modbus(frame, 2, 10)
        put_u16_le(frame, 12, crc)
        self.tx_sequence = (self.tx_sequence + 1) & 0xFF
        return frame

    def send_ball_state(self, tracker):
        """根据跟踪器状态发送BALL_STATE。

        参数tracker为BallTracker对象。有效坐标被限制到0～319；丢球时发送
        0xFFFF坐标和0置信度，确保STM32不会使用陈旧预测值继续控制。
        """
        flags = 0
        if tracker.is_initialized():
            flags |= FLAG_KALMAN_INITIALIZED

        if tracker.position_is_valid():
            filtered_x, filtered_y = tracker.get_position()
            x = int(round(clamp(filtered_x, 0.0, 319.0)))
            y = int(round(clamp(filtered_y, 0.0, 319.0)))
            confidence = int(round(clamp(tracker.last_confidence, 0.0, 1.0) * 1000.0))
            flags |= FLAG_POSITION_VALID
            if tracker.state == TRACK_MEASURED:
                flags |= FLAG_CURRENT_MEASURED
            elif tracker.state == TRACK_PREDICTED:
                flags |= FLAG_CURRENT_PREDICTED
        else:
            x = INVALID_COORDINATE
            y = INVALID_COORDINATE
            confidence = 0

        self.uart.write(self.build_frame(MSG_BALL_STATE, flags, x, y, confidence))

    def send_target_ack(self, target_x, target_y):
        """向STM32确认实际采用的目标点，坐标单位为像素。"""
        self.uart.write(self.build_frame(MSG_TARGET_ACK, 0, target_x, target_y, 0))

    def _discard_before_header(self):
        """丢弃接收缓存中位于第一个0xAA候选帧头之前的噪声字节。"""
        while len(self.rx_buffer) > 0 and self.rx_buffer[0] != PROTOCOL_SOF1:
            self.rx_buffer = self.rx_buffer[1:]

    def poll(self):
        """读取并解析当前已经到达的全部UART数据，返回有效帧列表。"""
        received = self.uart.read(64)
        if received:
            self.rx_buffer.extend(received)

        parsed_frames = []
        while True:
            self._discard_before_header()
            if len(self.rx_buffer) < 2:
                break

            if self.rx_buffer[1] != PROTOCOL_SOF2:
                self.rx_buffer = self.rx_buffer[1:]
                continue

            if len(self.rx_buffer) < PROTOCOL_FRAME_SIZE:
                break

            frame = self.rx_buffer[0:PROTOCOL_FRAME_SIZE]
            received_crc = get_u16_le(frame, 12)
            calculated_crc = crc16_modbus(frame, 2, 10)
            if received_crc == calculated_crc and frame[2] == PROTOCOL_VERSION:
                parsed_frames.append(frame)
                self.valid_frames += 1
                self.rx_buffer = self.rx_buffer[PROTOCOL_FRAME_SIZE:]
            else:
                self.crc_errors += 1
                # 只丢弃当前候选帧的第一个字节，后续字节中可能已经包含下一帧帧头。
                self.rx_buffer = self.rx_buffer[1:]

        # 异常噪声环境下限制缓存长度，防止长期运行后内存不断增加。
        if len(self.rx_buffer) > PROTOCOL_FRAME_SIZE * 4:
            self.rx_buffer = self.rx_buffer[-PROTOCOL_FRAME_SIZE:]
        return parsed_frames


class SteelBallDetector(AIBase):
    """YOLOv8钢球检测器，负责预处理、KPU推理和检测框后处理。"""

    def __init__(self, kmodel_path, debug_mode=0):
        """创建钢球检测器。

        参数kmodel_path为KModel绝对路径；debug_mode非0时输出AI处理耗时。
        模型、AI通道和显示坐标均固定为320×320，避免额外坐标映射误差。
        """
        super().__init__(kmodel_path, MODEL_INPUT_SIZE, AI_FRAME_SIZE, debug_mode)
        self.confidence_threshold = CONF_THRESH
        self.nms_threshold = NMS_THRESH
        self.max_boxes_num = MAX_BOXES_NUM
        self.model_input_size = MODEL_INPUT_SIZE
        self.ai_frame_size = AI_FRAME_SIZE
        self.display_size = DISPLAY_SIZE
        self.debug_mode = debug_mode

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self):
        """建立320x320到320x320的恒等预处理流程，不进行拉伸和补边。"""
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            self.ai2d.build(
                [1, 3, AI_FRAME_SIZE[1], AI_FRAME_SIZE[0]],
                [1, 3, MODEL_INPUT_SIZE[1], MODEL_INPUT_SIZE[0]],
            )

    def postprocess(self, results):
        """检查KPU输出形状，并把YOLO结果转换为[x1,y1,x2,y2,置信度]。"""
        with ScopedTiming("postprocess", self.debug_mode > 0):
            if not results or len(results) != 1:
                print("KPU输出数量异常:", 0 if not results else len(results))
                return []

            result = results[0]
            expected_channels = 4 + len(LABELS)
            if (
                len(result.shape) != 3
                or result.shape[0] != 1
                or result.shape[1] != expected_channels
                or result.shape[2] != EXPECTED_OUTPUT_POINTS
            ):
                print(
                    "KPU输出形状异常:",
                    result.shape,
                    "期望:",
                    (1, expected_channels, EXPECTED_OUTPUT_POINTS),
                )
                return []

            native_output = result[0].transpose().copy()
            det_res = aidemo.yolov8_det_postprocess(
                native_output,
                [AI_FRAME_SIZE[1], AI_FRAME_SIZE[0]],
                [MODEL_INPUT_SIZE[1], MODEL_INPUT_SIZE[0]],
                [DISPLAY_SIZE[1], DISPLAY_SIZE[0]],
                len(LABELS),
                self.confidence_threshold,
                self.nms_threshold,
                self.max_boxes_num,
            )

            if det_res is None or len(det_res) < 3 or len(det_res[0]) == 0:
                return []

            boxes, class_ids, scores = det_res
            detections = []
            for i in range(len(boxes)):
                x, y, w, h = boxes[i]
                x1 = max(0, min(319, int(round(x))))
                y1 = max(0, min(319, int(round(y))))
                x2 = max(0, min(319, int(round(x + w))))
                y2 = max(0, min(319, int(round(y + h))))
                if x2 > x1 and y2 > y1:
                    detections.append([x1, y1, x2, y2, float(scores[i])])
            return detections


def draw_cross(frame, x, y, color, radius=6, thickness=2):
    """在画面指定坐标绘制十字。

    frame为CanMV图像，x/y单位为像素，color为RGB颜色，radius为半径，
    thickness为线宽。中心点和端点都会限制在320×320画面范围内。
    """
    x = int(round(clamp(x, 0, 319)))
    y = int(round(clamp(y, 0, 319)))
    frame.draw_line(max(0, x - radius), y, min(319, x + radius), y,
                    color=color, thickness=thickness)
    frame.draw_line(x, max(0, y - radius), x, min(319, y + radius),
                    color=color, thickness=thickness)


def draw_detections(frame, detections, selected_detection, tracker,
                    target_x, target_y, fps, tx_sequence):
    """绘制检测框、原始点、滤波点、目标点和运行状态。

    detections为本帧全部候选框，selected_detection为关联成功的同一列表对象，
    tracker提供滤波坐标和状态；target_x/target_y为目标像素坐标，fps为当前帧率，
    tx_sequence为下一帧发送序号。函数只修改显示图像，不影响识别和UART通信。
    """
    green = (0, 255, 0)
    magenta = (255, 0, 255)
    yellow = (255, 255, 0)
    cyan = (0, 255, 255)
    red = (255, 0, 0)
    white = (255, 255, 255)

    for detection in detections:
        x1, y1, x2, y2 = detection[0:4]
        is_selected = detection is selected_detection
        color = magenta if is_selected else green
        thickness = 3 if is_selected else 1
        frame.draw_rectangle(
            x1, y1, x2 - x1, y2 - y1,
            color=color, thickness=thickness, fill=False
        )
        if DRAW_BOX_SCORE:
            text_y = y1 - BOX_FONT_SIZE - 1
            if text_y < SUMMARY_BAR_HEIGHT:
                text_y = y1 + 1
            frame.draw_string_advanced(
                x1, text_y, BOX_FONT_SIZE,
                "%.2f" % detection[4], color=color
            )

    if selected_detection is not None:
        raw_x = 0.5 * (selected_detection[0] + selected_detection[2])
        raw_y = 0.5 * (selected_detection[1] + selected_detection[3])
        frame.draw_circle(int(raw_x), int(raw_y), 4, color=yellow,
                          thickness=2, fill=False)

    draw_cross(frame, target_x, target_y, red, radius=7, thickness=2)

    if tracker.position_is_valid():
        filtered_x, filtered_y = tracker.get_position()
        draw_cross(frame, filtered_x, filtered_y, cyan, radius=5, thickness=2)
        frame.draw_line(
            int(target_x), int(target_y), int(filtered_x), int(filtered_y),
            color=white, thickness=1
        )
        error_x = int(round(target_x - filtered_x))
    else:
        error_x = 0

    if tracker.state == TRACK_MEASURED:
        state_text = "MEAS"
    elif tracker.state == TRACK_PREDICTED:
        state_text = "PRED"
    elif tracker.state == TRACK_RESET:
        state_text = "RESET"
    else:
        state_text = "LOST"

    frame.draw_string_advanced(
        2, 2, SUMMARY_FONT_SIZE,
        "%s E:%+d F:%.1f" % (state_text, error_x, fps), color=yellow
    )
    frame.draw_string_advanced(
        2, 20, SUMMARY_FONT_SIZE,
        "T:%d,%d C:%.2f SQ:%d"
        % (target_x, target_y, tracker.last_confidence, tx_sequence),
        color=yellow
    )


def print_status(detections, selected_detection, tracker, fps, protocol,
                 target_x, target_y):
    """低频打印识别、跟踪、目标和协议统计。

    所有参数均为当前帧只读状态。本函数只用于现场调试；通过调用频率限制避免
    逐帧打印占用过多CPU和串口时间。selected_detection仅用于详细框输出。
    """
    if tracker.position_is_valid():
        filtered_x, filtered_y = tracker.get_position()
        position_text = "x=%.1f y=%.1f" % (filtered_x, filtered_y)
    else:
        position_text = "position=invalid"

    print(
        "检测=%d 状态=%d %s 目标=(%d,%d) fps=%.2f tx=%d rx=%d crc_err=%d"
        % (
            len(detections), tracker.state, position_text,
            target_x, target_y, fps, protocol.tx_sequence,
            protocol.valid_frames, protocol.crc_errors,
        )
    )
    if PRINT_BOX_DETAILS:
        for i, detection in enumerate(detections):
            print(
                "  #%d x1=%d y1=%d x2=%d y2=%d conf=%.3f selected=%s"
                % (
                    i + 1, detection[0], detection[1],
                    detection[2], detection[3], detection[4],
                    "yes" if detection is selected_detection else "no",
                )
            )


def main():
    """初始化K230全部资源并持续执行识别、滤波、显示和双向UART通信。

    本函数没有参数和返回值。任何异常都会进入finally，依次释放模型、摄像头、
    显示、媒体缓存和UART，避免CanMV IDE再次运行脚本时资源仍被占用。
    """
    sensor = None
    detector = None
    uart = None
    display_started = False
    media_started = False

    try:
        # 配置K230 UART1引脚复用。UART对象必须在FPIOA设置之后创建。
        fpioa = FPIOA()
        fpioa.set_function(UART_TX_PIN, UART_TX_FUNCTION)
        fpioa.set_function(UART_RX_PIN, UART_RX_FUNCTION)
        uart = UART(UART_ID, UART_BAUDRATE)
        protocol = BalanceProtocol(uart)

        sensor = Sensor()
        sensor.reset()

        if ENABLE_DISPLAY:
            sensor.set_framesize(
                width=DISPLAY_SIZE[0], height=DISPLAY_SIZE[1], chn=CAM_CHN_ID_0
            )
            sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

        sensor.set_framesize(
            width=AI_FRAME_SIZE[0], height=AI_FRAME_SIZE[1], chn=CAM_CHN_ID_2
        )
        sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)

        if ENABLE_DISPLAY:
            Display.init(
                Display.VIRT,
                DISPLAY_SIZE[0],
                DISPLAY_SIZE[1],
                to_ide=True,
            )
            display_started = True

        MediaManager.init()
        media_started = True

        detector = SteelBallDetector(KMODEL_PATH, DEBUG_MODE)
        detector.config_preprocess()
        tracker = BallTracker()

        sensor.run()
        clock = time.clock()
        frame_count = 0
        previous_ms = get_monotonic_ms()
        target_x = DEFAULT_TARGET_X
        target_y = DEFAULT_TARGET_Y

        print("K230钢球识别、卡尔曼滤波和STM32通信程序已启动")
        print("模型:", KMODEL_PATH)
        print("图像/模型坐标: 320x320")
        print("UART1: GPIO3=TX GPIO4=RX 115200 8N1")

        while True:
            clock.tick()
            frame_count += 1

            # 先处理STM32发来的目标命令，保证目标点尽快在当前画面生效。
            for received_frame in protocol.poll():
                if received_frame[3] == MSG_SET_TARGET:
                    requested_x = get_u16_le(received_frame, 6)
                    requested_y = get_u16_le(received_frame, 8)
                    if requested_x <= 319 and requested_y <= 319:
                        target_x = requested_x
                        target_y = requested_y
                        protocol.send_target_ack(target_x, target_y)
                        print("收到新目标点: (%d,%d)" % (target_x, target_y))

            now_ms = get_monotonic_ms()
            elapsed_ms = get_elapsed_ms(now_ms, previous_ms)
            previous_ms = now_ms
            dt = clamp(elapsed_ms / 1000.0,
                       KALMAN_MIN_DT_SECONDS, KALMAN_MAX_DT_SECONDS)

            ai_frame = sensor.snapshot(chn=CAM_CHN_ID_2)
            ai_input = ai_frame.to_numpy_ref()
            detections = detector.run(ai_input)

            # 已建立轨迹时先预测，再用预测点选择与当前轨迹最接近的检测框。
            tracker.predict(dt)
            selected_detection = tracker.select_detection(detections)
            if selected_detection is not None:
                tracker.update_with_detection(selected_detection)
            else:
                tracker.update_without_detection()

            protocol.send_ball_state(tracker)
            fps = clock.fps()

            if ENABLE_DISPLAY and frame_count % DISPLAY_EVERY_N_FRAMES == 0:
                display_frame = sensor.snapshot(chn=CAM_CHN_ID_0)
                draw_detections(
                    display_frame, detections, selected_detection, tracker,
                    target_x, target_y, fps, protocol.tx_sequence
                )
                Display.show_image(display_frame)
                del display_frame

            if frame_count % PRINT_EVERY_N_FRAMES == 0:
                print_status(
                    detections, selected_detection, tracker, fps,
                    protocol, target_x, target_y
                )

            del ai_input
            del ai_frame

            if frame_count % GC_EVERY_N_FRAMES == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("用户停止程序")
    except Exception as error:
        sys.print_exception(error)
    finally:
        if detector is not None:
            try:
                detector.deinit()
            except Exception as error:
                print("检测器释放失败:", error)
            detector = None

        if sensor is not None:
            try:
                sensor.stop()
            except Exception as error:
                print("摄像头停止失败:", error)
            sensor = None

        if display_started:
            try:
                Display.deinit()
            except Exception as error:
                print("显示释放失败:", error)

        if media_started:
            try:
                MediaManager.deinit()
            except Exception as error:
                print("媒体内存释放失败:", error)

        if uart is not None:
            try:
                uart.deinit()
            except Exception:
                pass
            uart = None

        gc.collect()
        print("钢球识别程序资源已释放")


if __name__ == "__main__":
    main()
