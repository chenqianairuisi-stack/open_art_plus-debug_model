# ===========================================================================
# OpenART 绿色色块跟踪 + 地图数据通信程序
# ===========================================================================
# 运行流程说明：
# 1. 开机后初始化摄像头，固定曝光、增益和白平衡，避免自动调节导致颜色阈值漂移。
# 2. 主循环每帧采集一张图像，并统一做 gamma/亮度校正。
# 3. 串口收到主控请求地图时，暂停位姿跟踪，连续扫描到稳定地图后打包回发。
# 4. 没有地图发送任务时，跟踪器在上一帧位置附近搜索绿色色块：
#    - 用 TRACK_GREEN_*（绿+青两条阈值）merge 出一个完整车身 blob，阈值里不含黄色，所以贴黄箱也不糊框；
#    - 再按尺寸、填充率和宽高比做基础过滤，多个候选时偏向靠近上一帧中心的那个；
#    - 框直接用 blob 真实外接矩形，中心用 blob 质心（做轻平滑防抖）。
# 5. 找到绿色色块后，把像素中心通过四角标定映射成地图厘米坐标，并按主控请求回发位姿。
# 6. 调试顺序建议：先确认 FIXED_CORNERS，再在阈值编辑器里调干净 TRACK_GREEN_MAIN/TRACK_GREEN_CYAN，最后再微调搜索半径和抗抖参数。
# ===========================================================================

import sensor, time, ustruct, math, omv
from machine import UART

TRACK_ROI_MARGIN_PX = 10  # 绿色小车允许超出地图外框的像素余量；压边时框被截断就调大。

# ===========================================================================
# [1] 硬件与逻辑配置 (参数完全保留)
# ===========================================================================
DEBUG_MODE    = False      # 总调试开关：True 会打印地图预览、画更多辅助信息；正式跑车建议 False，减少帧率开销。
TRACK_DEBUG   = True     # 跟踪调试显示：True 会画搜索 ROI、绿色小车框和坐标文字。
TRACK_BLOB_DEBUG = False  # 画出 find_blobs 找到的原始绿色候选；灰框=被筛1掉，绿框=最终选中。
TRACK_DRAW_SEARCH_ROI = False # 画青色搜索范围；只想看最终绿色框时可设 False。
TRACK_DRAW_HEADING_ROIS = False  # 左右红色大框只是航向角采样区，默认不画，避免误看成跟踪框。

# 固定的四个角坐标 (QVGA 320x240)
# 地图外框四角，顺序固定为：左上、右上、右下、左下。
# 这四个点决定像素坐标到地图厘米坐标的映射；如果越靠边误差越大，优先重新标定这里。
FIXED_CORNERS = [(17, 24), (297, 24), (295, 216), (20, 216)]

# 画面亮度均匀化：OpenMV 原生自适应直方图均衡 (CLAHE)，让画面中心和边缘亮度更均匀。
# clip_limit 越大对比拉伸越强；过大会放大噪声，过小则均匀化效果不明显。
ILLUMINATION_DRAW_LABEL = True
CLAHE_CLIP_LIMIT = 10
BASE_GAMMA = 1.10
BASE_CONTRAST = 1.05
BASE_BRIGHTNESS = 0.00
ENABLE_GAMMA_CORR = False

# OpenARTPlus 当前画面中心过曝时，优先改这里：700 更暗，950 更亮
CAMERA_EXPOSURE_US = 500

CMD_REQ_MAP   = 0x10     # 主板请求地图数据的命令字。
CMD_REQ_POSE  = 0x11     # 主板请求小车位姿的命令字。
MSG_MAP_DATA  = 0x20     # OpenMV 返回"地图数据"的消息类型。
MSG_POSE_DATA = 0x21     # OpenMV 返回小车"位姿"的消息类型。
UART_ANY_SIGNAL_START_POSE = True
MAP_SEND_REPEAT_COUNT = 5
MAP_CAPTURE_SETTLE_FRAMES = 0   # 连续发车要求秒回：收到请求后不丢帧，当帧就开始扫描。
MAP_STABLE_CONFIRM_FRAMES = 10  # 连续扫到这么多帧地图完全一致，才认定地图稳定并打包发送。
MAP_FAST_REPLY_AFTER_FIRST_VALID = True
MAP_CAPTURE_MAX_SCAN_FRAMES = 3  # 最多扫这么多帧就兜底发出去，控制主控发信号到收到地图的最坏延迟。
GRID_SIZE_CM  = 20.0     # 每个地图格子的实际边长，单位 cm；输出坐标按这个比例换算。

# 阈值判定
# L 是亮度
# A 越小越偏绿、越大越偏红
# B 越小越偏蓝、越大越偏黄
T_BOX   = (71, 100, -37, -8, 26, 100)    # 黄色箱子 $
T_TGT   = (53, 79, 49, 96, -72, -31)     # 紫色/粉色目标 .
T_BOMB  = (36, 72, 30, 96, -23, 90)      # 红色炸弹 *
T_WALL  = (44, 100, -19, 45, -43, 14)     # 灰色墙壁 #

# 每个格子 ROI 内某颜色至少要有这么多“命中像素”才算识别到该颜色。
# 用像素计数代替 LAB 均值判定：格子边缘混进背景蓝/彩色边框时不会再把均值拉出阈值，
# 从根本上解决“黄色中心很清晰却时有时无”的漏标问题。ROI 为 12x12=144 像素。
CELL_MIN_MATCH_PIXELS = 15

# =====绿色色块跟踪--相比于之前的框选中整体绿色色块，锁定范围，这里做了简化=====
# 直接框 blob，使用的车身阈值：绿色 + 青色
# 关键思路：阈值里完全不含黄色，所以即使小车贴着黄色箱子，"merge"出来的"blob"也只会停在绿色边界--阈值本身就是黄色排除器
# 分离要点：
#   --黄色 B≥21，所以绿色那半把 B 上限压到 ~40，避免大面积重叠；
#   --青色那半 B 很负（偏蓝），全场独一无二，是最可靠的车身锚点。
# 这一套代码都是通过扩大阈值范围寻找到相似或者易误判色块的子母集，进行排斥和留存的操作

TRACK_GREEN_MAIN = (70, 100, -85, -41, -3, 92)    # 纯绿部分--B 上限 40 用来挡黄色。
TRACK_GREEN_CYAN = (71, 94, -52, -20, -29, 24)    # 青色部分
TRACK_GREEN_THRESH = [TRACK_GREEN_MAIN, TRACK_GREEN_CYAN]
TRACK_MERGE_MARGIN = 6     # 绿/青两半之间的接缝桥接距离；调大更容易合成一个完整框，太大会粘到旁边噪声
GREEN_NEAR_BIAS = 0.05     # 多个候选时，偏向靠近上一帧中心的 blob 的程度--置信程度

# 绿色小车 blob 基础筛选参数（作用于"merge"后的整车"blob"）
# 无畸变摄像头所看见的大小相近的色块判定条件--"形状是否正方形"
GREEN_MIN_PIXELS = 10             # 合并后车身 blob 的最少像素数
GREEN_MIN_AREA   = 16             # blob 外接矩形面积下限；配合 GREEN_MIN_PIXELS 去掉零散噪声
GREEN_MIN_SIDE   = 3              # 外接矩形最小宽/高
GREEN_MAX_SIDE   = 64             # 外接矩形最大宽/高硬上限
GREEN_MAX_RATIO  = 2.60           # 车身可能只露半块/边缘，不强求完整正方形
GREEN_MIN_FILL = 0.08             # 车身可能只露一部分，填充率不能设太严。
GREEN_MODEL_DEADBAND_PX = 1.5     # 中心轻微抖动时压住小跳：调大更稳，调小更跟手--本质运算逻辑就是更相信当前帧还是更相信历史帧
GREEN_MODEL_ALPHA = 0.75          # 绿色框中心平滑权重：越大越跟手，越小越稳--滤波补助，这里是一阶低通
GREEN_LOCAL_SEARCH_RADIUS_PX = 44 # 已锁定后优先在上一帧附近搜索，防止远处黄色噪声抢框
GREEN_REACQUIRE_SEARCH_RADIUS_PX = 82 # 短暂丢帧时逐步放大的搜索半径
GREEN_JUMP_GATE_PX = 34           # 锁定状态下单帧中心最大跳变，黄色抢框通常会超过这个距离
GREEN_LOST_HOLD_FRAMES = 5        # 短暂丢检时保留上一帧框和坐标，避免闪烁

# 航向角 min_corners() 形状校验：只有角点形状像一个完整方块时，才用它来更新 yaw
# 运动到后期陀螺仪会开始飘，需要依靠摄像头来完善
CORNER_SIDE_RATIO_MAX = 1.55      # 四条边最长/最短允许比例，调小更像正方形，太小会在边缘缺失时误拒绝
CORNER_SIZE_MIN_SCALE = 0.52      # 角点框平均边长下限，相对理论一格边长；过小认为不是完整小车
CORNER_SIZE_MAX_SCALE = 1.72      # 角点框平均边长上限，过大认为是边缘噪声合并出来的大框

# 航向角计算区域：只有小车中心进入这个红色 ROI 时才更新 yaw，其他区域不算航向角，节省帧率
HEADING_ROIS = [
    (10, 49, 56, 118),     # 左侧航向角更新区域
    (258, 58, 56, 118),    # 右侧航向角更新区域
]
HEADING_BUF_MAX = 35       # 偏航角（Yaw）使用车辆静止时的采样窗口--窗口越大，数值越平稳，但响应速度越慢
HEADING_MIN_SAMPLES = 8    # 在允许进行强力修剪（Trimming）之前，所需的最小有效偏航角样本数
HEADING_TRIM_SAMPLES = 5   # 当数据量足够后，截断平均值法（Trimmed Mean）将剔除相应数量的最高值和最低值样本
HEADING_RESET_DEG = 18.0   # 如果实际测量的车身角度变化超过此数值，则重置偏航角均值计算

# alpha-beta 位姿预测滤波。
# 它可以理解成轻量常速度卡尔曼：保存位置和速度，用当前测量修正预测。
# 可信测量默认直接输出；滤波预测主要用于拒绝跳点和短暂丢车时保持连续。
POSE_PREDICT_MS = 0               # 坐标前向预测时间；调试坐标一致性时设 0，避免同一位置被速度预测推出不同坐标。
POSE_ALPHA = 0.75                 # alpha-beta 的位置测量权重；越大越跟手，越小越稳但延时更明显。
POSE_BETA  = 0.23                 # alpha-beta 的速度修正权重；越大速度响应越快，但噪声也更容易进入速度估计。
POSE_DEADBAND_CM = 0.0            # 纯视觉坐标不做厘米级死区；否则 0.2cm 目标会被输出层直接破坏。
POSE_MEAS_GATE_CM = 18.0          # 厘米坐标测量门控基础值；测量离预测太远就不更新滤波，防止异常跳点。
POSE_GATE_SPEED_GAIN = 0.06       # 速度越快，厘米坐标门控自动放宽；避免高速运动时正常测量被误拒绝。
POSE_USE_HOMOGRAPHY = True        # True 用透视单应映射换算坐标；平面地图被相机斜拍时通常比双线性更准。
POSE_OUTPUT_MEASUREMENT = True    # True 时有可信测量就直接输出当前帧坐标，避免滤波/预测导致同一位置坐标漂移。
POSE_X_CELLS = 14.0               # 输出 X 对应地图横向 14 格：左小右大。
POSE_Y_CELLS = 10.0               # 输出 Y 对应地图纵向 10 格：上小下大。
POSE_X_MIN_CM = GRID_SIZE_CM      # 内场左上角 X 物理坐标；坐标系定义，不是误差补偿。
POSE_Y_MIN_CM = GRID_SIZE_CM      # 内场左上角 Y 物理坐标；坐标系定义，不是误差补偿。
POSE_X_MAX_CM = (POSE_X_CELLS + 1.0) * GRID_SIZE_CM
POSE_Y_MAX_CM = (POSE_Y_CELLS + 1.0) * GRID_SIZE_CM
POSE_OUTPUT_MARGIN_CM = 0.0       # 显示/发送坐标不允许越出地图；边缘处坐标会贴住真实边界，不再跑到 0~20cm 缓冲区。
# 车体中心真正能到达的范围比地图四角再内缩半格车身：地图四角映射成 (20,20)~(300,220)，
# 但小车贴墙时中心离墙仍有半个车身，所以左上中心=(30,30)、右下中心=(290,210)。
# 输出按这个“中心可达范围”裁剪，能直接消除像 290.7 这种越界读数。
POSE_CENTER_INSET_CM = GRID_SIZE_CM * 0.5
# 中心坐标距离可达边界小于这个带宽时，吸附到边界值：
#   --消除压边/沿边运动时垂直方向 ±2cm 的像素质心抖动（如底边 Y 在 208~210 间跳）；
#   --让贴边时坐标精确等于理论值（30/210/30/290）。
# 设 0 可关闭吸附；调大吸附更“硬”，调小则更接近原始测量。当前 2.5cm 略大于观测到的 ~2cm 抖动幅度。
POSE_EDGE_SNAP_CM = 1.5
POSE_RESET_MARGIN_CM = GRID_SIZE_CM * 2.0    # 预测位置超过这个缓冲就认为滤波器已经跑飞。
POSE_MAX_SPEED_CM_S = 420.0       # 位姿滤波速度上限；防止一帧坏测量把前向预测打到几万厘米。

uart = UART(12, 115200)

# ===========================================================================
# [2] 硬件初始化
# ===========================================================================
def init_camera():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time = 5)
    # 固定曝光/增益/白平衡，避免相机自动调节导致绿色阈值和地图阈值漂移。
    # 如果环境光整体变了，优先重新调颜色阈值或曝光时间，不建议打开自动模式。
    sensor.set_auto_exposure(False, CAMERA_EXPOSURE_US)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    omv.disable_fb(False)

# ===========================================================================
# [3] 数学工具
# ===========================================================================
def get_inverse_bilinear(corners, px, py):
    tl, tr, br, bl = corners
    ax = tl[0]
    ay = tl[1]
    bx = tr[0] - tl[0]
    by = tr[1] - tl[1]
    cx = bl[0] - tl[0]
    cy = bl[1] - tl[1]
    dx = tl[0] - tr[0] + br[0] - bl[0]
    dy = tl[1] - tr[1] + br[1] - bl[1]

    det0 = bx * cy - cx * by
    if abs(det0) < 1e-6:
        return 0.5, 0.5

    # 先用仿射逆解给 Newton 迭代一个初值，再求真实四边形双线性逆映射。
    # 这样地图四角不是完美矩形时，坐标也比简单线性比例更准。
    # 返回的 u/v 都被限制在 0~1，避免小车靠近边界时坐标跑出地图。
    u = (cy * (px - ax) - cx * (py - ay)) / det0
    v = (-by * (px - ax) + bx * (py - ay)) / det0

    for i in range(4):
        ex = ax + bx * u + cx * v + dx * u * v - px
        ey = ay + by * u + cy * v + dy * u * v - py
        j11 = bx + dx * v
        j12 = cx + dx * u
        j21 = by + dy * v
        j22 = cy + dy * u
        det = j11 * j22 - j12 * j21
        if abs(det) < 1e-6:
            break
        du = (j22 * ex - j12 * ey) / det
        dv = (-j21 * ex + j11 * ey) / det
        u -= du
        v -= dv

    if u < 0.0: u = 0.0
    elif u > 1.0: u = 1.0
    if v < 0.0: v = 0.0
    elif v > 1.0: v = 1.0
    return u, v

def solve_linear_8(a, y):
    # 8x8 高斯消元，只在初始化时跑一次，用于求图像四边形到地图矩形的透视单应矩阵。
    n = 8
    for col in range(n):
        pivot = col
        best = abs(a[col][col])
        for r in range(col + 1, n):
            v = abs(a[r][col])
            if v > best:
                best = v
                pivot = r
        if best < 1e-9:
            return None
        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]
            y[col], y[pivot] = y[pivot], y[col]

        inv = 1.0 / a[col][col]
        for c in range(col, n):
            a[col][c] *= inv
        y[col] *= inv

        for r in range(n):
            if r == col:
                continue
            f = a[r][col]
            if abs(f) < 1e-12:
                continue
            for c in range(col, n):
                a[r][c] -= f * a[col][c]
            y[r] -= f * y[col]
    return y

def build_homography(src_pts, dst_pts):
    # 求解 image(x,y) -> target(a,b) 的单应矩阵：
    # a=(h0*x+h1*y+h2)/(h6*x+h7*y+1), b=(h3*x+h4*y+h5)/(h6*x+h7*y+1)
    a = []
    y = []
    for i in range(4):
        x = float(src_pts[i][0])
        yy = float(src_pts[i][1])
        tx = float(dst_pts[i][0])
        ty = float(dst_pts[i][1])
        a.append([x, yy, 1.0, 0.0, 0.0, 0.0, -tx * x, -tx * yy])
        y.append(tx)
        a.append([0.0, 0.0, 0.0, x, yy, 1.0, -ty * x, -ty * yy])
        y.append(ty)
    return solve_linear_8(a, y)

def apply_homography(h, px, py):
    den = h[6] * px + h[7] * py + 1.0
    if abs(den) < 1e-6:
        return None
    x = (h[0] * px + h[1] * py + h[2]) / den
    y = (h[3] * px + h[4] * py + h[5]) / den
    return x, y

def build_image_to_pose_homography(corners):
    # 四角直接对应真实厘米坐标：X 左小右大，Y 上小下大。
    targets = (
        (POSE_X_MIN_CM, POSE_Y_MIN_CM),
        (POSE_X_MAX_CM, POSE_Y_MIN_CM),
        (POSE_X_MAX_CM, POSE_Y_MAX_CM),
        (POSE_X_MIN_CM, POSE_Y_MAX_CM)
    )
    return build_homography(corners, targets)

# ===========================================================================
# [4] 追踪器 (直接提取绿色块位置)
# ===========================================================================
# ROI / 几何辅助：这些小函数把“点是否在区域内、区域如何扩展”统一起来，
# 后面的搜索框、航向角区域和调试绘制都会复用，避免每个地方重复写边界判断。
def in_roi(px, py, roi):
    x, y, w, h = roi
    return x <= px <= (x + w) and y <= py <= (y + h)

def roi_index(px, py, rois):
    for i in range(len(rois)):
        if in_roi(px, py, rois[i]):
            return i
    return -1

def expand_roi(roi, margin):
    # 以地图外框为基础向四周扩一圈，用来允许小车压边或轻微出框时仍能被搜索到。
    # 返回值仍限制在 QVGA 画面范围内，避免 OpenMV 的 ROI 越界。
    x, y, w, h = roi
    x1 = int(x - margin)
    y1 = int(y - margin)
    x2 = int(x + w + margin)
    y2 = int(y + h + margin)
    if x1 < 0: x1 = 0
    if y1 < 0: y1 = 0
    if x2 > 320: x2 = 320
    if y2 > 240: y2 = 240
    return (x1, y1, x2 - x1, y2 - y1)

def clamp_roi(cx, cy, half, bounds):
    # 根据预测中心生成局部搜索框，并限制在地图范围内。
    # 局部搜索比全图搜索快，也能降低误选远处绿色噪声的概率。
    bx, by, bw, bh = bounds
    x1 = int(cx - half)
    y1 = int(cy - half)
    x2 = int(cx + half)
    y2 = int(cy + half)

    if x1 < bx: x1 = bx
    if y1 < by: y1 = by
    if x1 > bx + bw - 2: x1 = bx + bw - 2
    if y1 > by + bh - 2: y1 = by + bh - 2

    if x2 > bx + bw: x2 = bx + bw
    if y2 > by + bh: y2 = by + bh
    if x2 <= x1: x2 = x1 + 2
    if y2 <= y1: y2 = y1 + 2
    return (x1, y1, x2 - x1, y2 - y1)

# 角度辅助：统一航向角的表示，解决 0/360 跳变和车身边线前后等价的问题。
def norm360(a):
    a = a % 360.0
    if a < 0:
        a += 360.0
    return a

def sorted_copy(vals):
    out = []
    for v in vals:
        out.append(v)
    out.sort()
    return out

def corners_center(corners):
    sx, sy = 0.0, 0.0
    for p in corners:
        sx += p[0]
        sy += p[1]
    return sx * 0.25, sy * 0.25

def soft_deadband(last_v, target_v, deadband):
    # 软死区：很小的跳动直接压住，超过死区的变化只削掉 deadband 这一段。
    # 相比硬锁死，它不会在真正移动时产生明显卡顿。
    d = target_v - last_v
    if abs(d) <= deadband:
        return last_v
    if d > 0:
        return target_v - deadband
    return target_v + deadband

def clamp_value(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

# 位姿范围辅助：把像素测量换算到地图厘米坐标后，用这些函数做边界保护。
def pose_bounds(margin):
    x_min = POSE_X_MIN_CM - margin
    x_max = POSE_X_MAX_CM + margin
    y_min = POSE_Y_MIN_CM - margin
    y_max = POSE_Y_MAX_CM + margin
    return x_min, x_max, y_min, y_max

def pose_in_bounds(x, y, margin):
    x_min, x_max, y_min, y_max = pose_bounds(margin)
    return x_min <= x <= x_max and y_min <= y <= y_max

def clamp_pose(x, y, margin):
    x_min, x_max, y_min, y_max = pose_bounds(margin)
    return clamp_value(x, x_min, x_max), clamp_value(y, y_min, y_max)

# 车体中心可达范围：在地图四角对应坐标基础上四周各内缩半个车身。
def pose_center_bounds():
    x_min = POSE_X_MIN_CM + POSE_CENTER_INSET_CM
    x_max = POSE_X_MAX_CM - POSE_CENTER_INSET_CM
    y_min = POSE_Y_MIN_CM + POSE_CENTER_INSET_CM
    y_max = POSE_Y_MAX_CM - POSE_CENTER_INSET_CM
    return x_min, x_max, y_min, y_max

def snap_to_edge(v, lo, hi, band):
    # 先裁进可达范围（消除像 290.7 这种越界读数），
    # 再把贴近某条边界 band 以内的值吸附到边界值（压住沿边运动时垂直方向的像素抖动）。
    v = clamp_value(v, lo, hi)
    if band > 0.0:
        if v - lo <= band:
            return lo
        if hi - v <= band:
            return hi
    return v

def finalize_pose_xy(x, y):
    # 输出坐标的最后一道处理：把车体中心限制在“中心可达范围”内并做边界吸附。
    # 角点处 x、y 同时被吸附，沿边运动时只有垂直该边的坐标被钉死，平行方向照常变化。
    x_min, x_max, y_min, y_max = pose_center_bounds()
    fx = snap_to_edge(x, x_min, x_max, POSE_EDGE_SNAP_CM)
    fy = snap_to_edge(y, y_min, y_max, POSE_EDGE_SNAP_CM)
    return fx, fy

class UltimateTracker:
    def __init__(self, corners):
        # 跟踪器的职责：每帧找绿色小车，把像素中心映射成地图厘米坐标，并维护 yaw。
        # 初始化先根据地图四角推导搜索范围和尺寸门限，再准备像素/位姿/航向角滤波状态。
        self.corners = corners
        self.pose_h = build_image_to_pose_homography(corners) if POSE_USE_HOMOGRAPHY else None
        xs, ys = [p[0] for p in corners], [p[1] for p in corners]
        # map_roi 是地图本体范围，track_roi 在它外面放一点余量，search_roi 每帧会在两者之间切换。
        self.map_roi = (min(xs), min(ys), max(xs)-min(xs), max(ys)-min(ys))
        self.track_roi = expand_roi(self.map_roi, TRACK_ROI_MARGIN_PX)
        self.search_roi = self.track_roi
        # 根据标定地图尺寸估算“一格”在图像中的像素边长；绿色 blob 只作锚点，画框再套固定一格大小。
        self.green_side_px = (self.map_roi[2] / 14.0 + self.map_roi[3] / 10.0) * 0.5
        self.green_min_side = GREEN_MIN_SIDE
        self.green_max_side = GREEN_MAX_SIDE
        self.corner_min_side = self.green_side_px * CORNER_SIZE_MIN_SCALE
        self.corner_max_side = self.green_side_px * CORNER_SIZE_MAX_SCALE

        # x/y/yaw 是最终发给主板的输出；f/v 是厘米坐标滤波状态，i/iv 是像素坐标滤波状态。
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.fx, self.fy = 0.0, 0.0
        self.vx, self.vy = 0.0, 0.0
        self.ix, self.iy = 0.0, 0.0
        self.ivx, self.ivy = 0.0, 0.0

        # lost 只记录连续未检测到绿色的帧数；不参与预测搜索。
        self.have_pose = False
        self.have_img = False
        self.have_yaw = False
        self.lost = 0
        self.last_ms = time.ticks_ms()
        self.corners_px = None
        self.yaw_zone = -1
        self.yaw_samples = []
        self.found = False
        self.output_ready = False
        self.debug_best_blob = None
        self.have_model_center = False
        self.model_cx = 0.0
        self.model_cy = 0.0

    def _pixel_to_pose_cm(self, px, py):
        # 优先使用像素到真实厘米坐标的透视映射；矩阵异常时退回同一几何定义的双线性映射。
        if self.pose_h is not None:
            xy = apply_homography(self.pose_h, px, py)
            if xy is not None:
                x, y = xy
                return clamp_pose(x, y, 0.0)
        u, v = get_inverse_bilinear(self.corners, px, py)
        x = POSE_X_MIN_CM + u * (POSE_X_MAX_CM - POSE_X_MIN_CM)
        y = POSE_Y_MIN_CM + v * (POSE_Y_MAX_CM - POSE_Y_MIN_CM)
        return x, y

    def _next_dt(self):
        # 计算相邻两帧的时间间隔，供 alpha-beta 预测速度使用。
        # dt 做上下限保护，防止偶发卡顿让速度估计瞬间炸掉。
        now = time.ticks_ms()
        dt_ms = time.ticks_diff(now, self.last_ms)
        self.last_ms = now
        if dt_ms < 1:
            dt_ms = 1
        elif dt_ms > 250:
            dt_ms = 250
        return dt_ms * 0.001

    def _num(self, v, fallback=0.0):
        # OpenMV 的某些 API 会返回 list/tuple 点坐标；参与预测前统一取成数字。
        try:
            return float(v)
        except Exception:
            try:
                return float(v[0])
            except Exception:
                return fallback

    def _dt_num(self, dt):
        dt = self._num(dt, 0.001)
        if dt < 0.001:
            return 0.001
        if dt > 0.250:
            return 0.250
        return dt

    def _last_locked_center(self):
        if self.have_model_center:
            return self.model_cx, self.model_cy
        if self.have_img:
            return self.ix, self.iy
        if self.corners_px is not None:
            return corners_center(self.corners_px)
        return None

    def _set_search_roi(self, dt):
        # 已锁定时先搜上一帧附近，避免旁边黄色块/远处噪声参与竞争；丢失一小段时间后再放回全图。
        center = self._last_locked_center()
        if center is not None and self.lost <= GREEN_LOST_HOLD_FRAMES:
            half = GREEN_LOCAL_SEARCH_RADIUS_PX
            if self.lost > 0:
                half += self.lost * self.green_side_px * 0.65
                if half > GREEN_REACQUIRE_SEARCH_RADIUS_PX:
                    half = GREEN_REACQUIRE_SEARCH_RADIUS_PX
            self.search_roi = clamp_roi(center[0], center[1], half, self.track_roi)
        else:
            self.search_roi = self.track_roi

    def _valid_blob_reason(self, b):
        # 核心绿色 blob 只负责提供目标锚点；完整框由固定模型生成。
        w, h = b.w(), b.h()
        if w < self.green_min_side or h < self.green_min_side:
            return "small"
        if w > self.green_max_side or h > self.green_max_side:
            return "L%d,%d/%d" % (w, h, self.green_max_side)
        area = b.area()
        if area < GREEN_MIN_AREA or b.pixels() < GREEN_MIN_PIXELS:
            return "few"
        ratio = float(w) / float(h)
        if ratio < 1.0:
            ratio = 1.0 / ratio
        if ratio > GREEN_MAX_RATIO:
            return "ratio"
        if (float(b.pixels()) / float(area)) < GREEN_MIN_FILL:
            return "fill"
        return "ok"

    def _stabilize_model_center(self, cx, cy):
        # 阈值边缘轻微抖动时，死区内保持上一中心；超过死区后做轻量平滑，压住核心/兜底阈值切换造成的跳动。
        cx, cy = float(cx), float(cy)
        if self.have_model_center:
            dx = cx - self.model_cx
            dy = cy - self.model_cy
            dist2 = dx * dx + dy * dy
            if GREEN_MODEL_DEADBAND_PX > 0.0 and dist2 <= GREEN_MODEL_DEADBAND_PX * GREEN_MODEL_DEADBAND_PX:
                return self.model_cx, self.model_cy
            cx = self.model_cx + dx * GREEN_MODEL_ALPHA
            cy = self.model_cy + dy * GREEN_MODEL_ALPHA
        self.model_cx, self.model_cy = cx, cy
        self.have_model_center = True
        return cx, cy

    def _find_car_blob(self, img):
        # 直接找“绿+青合并后的车身 blob”，框就是它真实的外接矩形，天然紧贴绿色。
        # 阈值里没有黄色：merge 只会合并绿/青像素，黄色像素根本不会进来，
        # 所以贴着黄箱子时框也只停在绿色边界，无需任何额外的黄色排除逻辑。
        try:
            blobs = img.find_blobs(TRACK_GREEN_THRESH,
                                   roi=self.search_roi,
                                   pixels_threshold=GREEN_MIN_PIXELS,
                                   area_threshold=GREEN_MIN_AREA,
                                   merge=True,
                                   margin=TRACK_MERGE_MARGIN)
        except Exception:
            return None

        last = self._last_locked_center()
        best = None
        best_score = None
        for b in blobs:
            reason = self._valid_blob_reason(b)
            if reason != "ok":
                self.debug_blobs.append((b.rect(), reason))
                continue
            cx, cy = float(b.cx()), float(b.cy())
            score = -float(b.pixels())          # 同样合格时，偏向像素更多、更完整的车身。
            if last is not None:
                dx = cx - last[0]
                dy = cy - last[1]
                dist2 = dx * dx + dy * dy
                gate = GREEN_JUMP_GATE_PX + self.lost * self.green_side_px * 0.75
                # 锁定状态下，离上一帧太远的 blob 多半是远处噪声/黄绿误检，直接跳过。
                if self.lost <= GREEN_LOST_HOLD_FRAMES and dist2 > gate * gate:
                    self.debug_blobs.append((b.rect(), "jump"))
                    continue
                score += dist2 * GREEN_NEAR_BIAS   # 轻微偏向靠近上一帧中心的候选。
            if best is None or score < best_score:
                best = b
                best_score = score
        return best

    def _update_img_filter(self, cx, cy, dt):
        # 视觉预测已关闭：图像中心只记录当前帧检测结果，不估计图像速度。
        dt = self._dt_num(dt)
        cx = self._num(cx)
        cy = self._num(cy)
        self.ix, self.iy = cx, cy
        self.ivx, self.ivy = 0.0, 0.0
        self.have_img = True

    def _limit_pose_velocity(self):
        # 这里限制的是滤波器估计出来的速度，不是控制小车；作用是防止坏测量把预测点带飞。
        speed = math.sqrt(self.vx * self.vx + self.vy * self.vy)
        if speed > POSE_MAX_SPEED_CM_S and speed > 0.001:
            scale = POSE_MAX_SPEED_CM_S / speed
            self.vx *= scale
            self.vy *= scale

    def _update_pose_filter(self, rx, ry, dt):
        # 地图厘米坐标的 alpha-beta 滤波和前向补偿。
        # 先用当前测量修正位置/速度；输出可选择直接使用当前可信测量，避免调试框准但坐标滞后。
        if POSE_OUTPUT_MEASUREMENT:
            ox, oy = clamp_pose(rx, ry, POSE_OUTPUT_MARGIN_CM)
            ox, oy = finalize_pose_xy(ox, oy)
            self.x, self.y = ox, oy
            self.fx, self.fy = ox, oy
            self.vx, self.vy = 0.0, 0.0
            self.have_pose = True
            self.output_ready = True
            return

        accepted = True
        if not self.have_pose:
            self.fx, self.fy = rx, ry
            self.vx, self.vy = 0.0, 0.0
            self.have_pose = True
        else:
            px = self.fx + self.vx * dt
            py = self.fy + self.vy * dt
            if not pose_in_bounds(px, py, POSE_RESET_MARGIN_CM):
                # 重新全图找回小车，或预测已经跑出地图太远时，直接相信当前测量并清速度。
                self.fx, self.fy = rx, ry
                self.vx, self.vy = 0.0, 0.0
            else:
                ex = rx - px
                ey = ry - py
                speed = math.sqrt(self.vx * self.vx + self.vy * self.vy)
                gate = POSE_MEAS_GATE_CM + speed * POSE_GATE_SPEED_GAIN
                # 测量点离预测太远时视为异常帧，不让它污染位置和速度。
                if (ex * ex + ey * ey) <= gate * gate:
                    self.fx = px + POSE_ALPHA * ex
                    self.fy = py + POSE_ALPHA * ey
                    self.vx += (POSE_BETA * ex) / dt
                    self.vy += (POSE_BETA * ey) / dt
                    self._limit_pose_velocity()
                else:
                    accepted = False
                    self.fx, self.fy = px, py
                    self.vx *= 0.70
                    self.vy *= 0.70

        if POSE_OUTPUT_MEASUREMENT and accepted:
            ox, oy = clamp_pose(rx, ry, POSE_OUTPUT_MARGIN_CM)
            ox, oy = finalize_pose_xy(ox, oy)
            self.x, self.y = ox, oy
            self.output_ready = True
            return

        lead = POSE_PREDICT_MS * 0.001
        ox = self.fx + self.vx * lead
        oy = self.fy + self.vy * lead
        ox, oy = clamp_pose(ox, oy, POSE_OUTPUT_MARGIN_CM)
        ox, oy = finalize_pose_xy(ox, oy)
        if not self.output_ready:
            self.x, self.y = ox, oy
            self.output_ready = True
        else:
            self.x = soft_deadband(self.x, ox, POSE_DEADBAND_CM)
            self.y = soft_deadband(self.y, oy, POSE_DEADBAND_CM)

    def _order_corners(self, corners, cx, cy):
        # 按角点相对中心的极角排序，保证后续边长、对角线和 yaw 计算使用同一种顺序。
        return sorted(corners, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))

    def _corner_yaw_shape_ok(self, pts):
        # 航向角只需要角点形状大致像一个方块，不要求中心和上一帧完全一致。
        sides = []
        for i in range(4):
            p1 = pts[i]
            p2 = pts[(i + 1) % 4]
            dx = p1[0] - p2[0]
            dy = p1[1] - p2[1]
            sides.append(math.sqrt(dx * dx + dy * dy))
        min_s, max_s = min(sides), max(sides)
        if min_s < 0.001:
            return False
        mean_s = (sides[0] + sides[1] + sides[2] + sides[3]) * 0.25
        if mean_s < self.corner_min_side or mean_s > self.corner_max_side:
            return False
        if (max_s / min_s) > CORNER_SIDE_RATIO_MAX:
            return False
        return True

    def _heading_sample_from_corners(self, pts, zone):
        # 左侧/右侧航向 ROI 的参考方向不同：
        # 左侧优先找接近 90 度的边，右侧优先找接近 270 度的边。
        # 返回值就是这条边在图像坐标系里的实际方向角。
        best_score = 1000000000.0
        best_yaw = None
        for i in range(4):
            p1 = pts[i]
            p2 = pts[(i + 1) % 4]
            dx0 = p2[0] - p1[0]
            dy0 = p2[1] - p1[1]

            for sign in (-1.0, 1.0):
                dx = dx0 * sign
                dy = dy0 * sign

                # 图像坐标系方向：上=0，右=90，下=180，左=270。
                h = norm360(math.degrees(math.atan2(dx, -dy)))

                if zone == 0:
                    # 左侧 ROI：只接受 0~180 度范围内、最接近 90 度的边。
                    if h <= 180.0:
                        score = abs(h - 90.0)
                        if score < best_score:
                            best_score = score
                            best_yaw = h
                else:
                    # 右侧 ROI：只接受 180~360 度范围内、最接近 270 度的边。
                    if h >= 180.0:
                        score = abs(h - 270.0)
                        if score < best_score:
                            best_score = score
                            best_yaw = h

        return best_yaw

    def _stable_yaw_from_samples(self):
        # 航向角会受角点抖动影响，样本足够时先去掉两端离群值，再求稳定平均。
        n = len(self.yaw_samples)
        if n == 0:
            return None

        kept = sorted_copy(self.yaw_samples)
        if len(kept) > (HEADING_TRIM_SAMPLES * 2 + HEADING_MIN_SAMPLES):
            kept = kept[HEADING_TRIM_SAMPLES:len(kept) - HEADING_TRIM_SAMPLES]

        s = 0.0
        for v in kept:
            s += v
        return s / len(kept)

    def _update_yaw_if_needed(self, b, cx, cy, dt):
        zone = roi_index(cx, cy, HEADING_ROIS)
        if zone < 0:
            return
        try:
            pts = b.min_corners()
        except Exception:
            return
        if pts is None or len(pts) != 4:
            return
        pts = self._order_corners(pts, cx, cy)
        if not self._corner_yaw_shape_ok(pts):
            return

        if zone != self.yaw_zone:
            self.yaw_zone = zone
            self.yaw_samples = []

        sample = self._heading_sample_from_corners(pts, zone)
        if sample is None:
            return

        if self.have_yaw and self.yaw_zone == zone:
            if abs(sample - self.yaw) > HEADING_RESET_DEG:
                self.yaw_samples = []

        self.yaw_samples.append(sample)
        if len(self.yaw_samples) > HEADING_BUF_MAX:
            self.yaw_samples.pop(0)

        yaw = self._stable_yaw_from_samples()
        if yaw is not None:
            self.yaw = yaw
            self.have_yaw = True

    def update(self, img):
        # 每帧跟踪流程（简化版）：
        # 1. 锁定后优先在上一帧附近找“绿+青”合并 blob；
        # 2. 显示框 = blob 真实外接矩形（紧贴绿色，移动/变形/贴黄色都跟手）；
        # 3. 坐标中心 = blob 像素质心，做一层轻平滑防抖后映射成厘米；
        # 4. 短暂丢帧保留上一帧中心，超过保持帧数后再放大搜索范围重捕获。
        dt = self._next_dt()
        self._set_search_roi(dt)

        self.debug_blobs = []
        self.debug_best_blob = None

        b = self._find_car_blob(img)
        if b:
            cx, cy = float(b.cx()), float(b.cy())
            self.found = True

            # 显示框直接用当前帧真实外接矩形；坐标中心做轻平滑，只压阈值边缘的细小抖动。
            self.debug_best_blob = b.rect()
            scx, scy = self._stabilize_model_center(cx, cy)
            self.corners_px = None

            self._update_img_filter(scx, scy, dt)
            rx, ry = self._pixel_to_pose_cm(scx, scy)
            self._update_pose_filter(rx, ry, dt)
            self._update_yaw_if_needed(b, cx, cy, dt)

            self.lost = 0
            return True

        self.lost += 1
        self.found = False
        if self.lost <= GREEN_LOST_HOLD_FRAMES and self._last_locked_center() is not None:
            return False
        self.corners_px = None
        self.have_model_center = False
        return False

# ===========================================================================
# [5] 串口与地图扫描 (核心逻辑修改点)
# ===========================================================================
def send_packet(msg_type, payload):
    # 所有串口消息统一使用 AA 55 + 类型 + 长度 + payload + 校验和，主板只需要解析这一种帧格式。
    length = len(payload)
    checksum = (msg_type + length + sum(payload)) & 0xFF
    header = bytearray([0xAA, 0x55, msg_type, length])
    uart.write(header + payload + bytearray([checksum]))

def send_map_response(map_bits, boxes, targets, bombs):
    # 地图包由障碍位图 + 箱子/目标/炸弹坐标组成；坐标都已经是主板需要的格点坐标。
    payload = bytearray(map_bits)
    box_cnt = min(len(boxes), 15)
    # 主板解析缓冲区是 64 字节。
    # ART1 地图包长度 = 25 + 箱子数 + 目标数 + 炸弹数；目标数量跟随箱子数量。
    bomb_room = 64 - 25 - box_cnt * 2
    if bomb_room < 0:
        bomb_room = 0
    bomb_cnt = min(len(bombs), 15, bomb_room)
    payload.append((box_cnt << 4) | bomb_cnt)
    for k in range(box_cnt):
        payload.append((boxes[k][0] << 4) | boxes[k][1])
    for k in range(box_cnt):
        if k < len(targets):
            payload.append((targets[k][0] << 4) | targets[k][1])
        else:
            payload.append(0x00)
    for k in range(bomb_cnt):
        payload.append((bombs[k][0] << 4) | bombs[k][1])
    send_packet(MSG_MAP_DATA, payload)

def send_pose_response(pose_data):
    # 位姿包只发三个 float：厘米坐标 x、厘米坐标 y、航向角 yaw。
    payload = ustruct.pack('<fff', pose_data[0], pose_data[1], pose_data[2])
    send_packet(MSG_POSE_DATA, payload)

# 串口接收状态机：逐字节识别 AA 55 帧头，校验通过后把命令转成 pending 标志。
# pending 标志让主循环在同一套节奏里处理命令，避免在读串口时直接做耗时扫描。
uart_state = 0
rx_msg_type, rx_length = 0, 0
rx_payload = bytearray()
pending_map_cmd = False
pending_pose_cmd = False

def poll_uart_commands():
    global uart_state, rx_msg_type, rx_length, rx_payload, pending_map_cmd, pending_pose_cmd
    # 非阻塞读取：本帧有多少字节就吃多少字节，不因为等待串口而卡住图像处理。
    while uart.any():
        b = uart.readchar()
        if uart_state == 0 and b == CMD_REQ_MAP:
            pending_map_cmd = True
            uart_state = 0
        elif uart_state == 0 and b == CMD_REQ_POSE:
            pending_pose_cmd = True
            uart_state = 0
        elif uart_state == 0 and b == 0xAA:
            uart_state = 1
        elif uart_state == 0 and UART_ANY_SIGNAL_START_POSE:
            pending_pose_cmd = True
            uart_state = 0
        elif uart_state == 1:
            if b == 0x55:
                uart_state = 2
            else:
                if UART_ANY_SIGNAL_START_POSE:
                    pending_pose_cmd = True
                uart_state = 0
        elif uart_state == 2:
            rx_msg_type = b
            uart_state = 3
        elif uart_state == 3:
            rx_length = b
            rx_payload = bytearray()
            uart_state = 4 if rx_length > 0 else 5
        elif uart_state == 4:
            rx_payload.append(b)
            if len(rx_payload) == rx_length:
                uart_state = 5
        elif uart_state == 5:
            chk = (rx_msg_type + rx_length + sum(rx_payload)) & 0xFF
            if b == chk:
                if rx_msg_type == CMD_REQ_MAP:
                    pending_map_cmd = True
                elif rx_msg_type == CMD_REQ_POSE:
                    pending_pose_cmd = True
                else:
                    # 纯视觉方案下，未知合法请求默认回位姿，避免主控频繁误触发地图扫描。
                    pending_pose_cmd = True
            uart_state = 0
    # 同一轮如果累计收到多个命令，地图请求优先；地图发完后主板再请求位姿即可恢复跟踪。
    if pending_map_cmd:
        pending_map_cmd = False
        return CMD_REQ_MAP
    if pending_pose_cmd:
        pending_pose_cmd = False
        return CMD_REQ_POSE
    return None

def generate_inner_rois(corners):
    # 根据地图四角生成 10x14 个内部格子的采样 ROI。
    # 每个 ROI 取格子中心附近 12x12 像素，避开格子边线，提高颜色统计稳定性。
    tl, tr, br, bl = corners
    rois = []
    for r in range(10):
        row = []
        for c in range(14):
            u, v = (c + 0.5) / 14.0, (r + 0.5) / 10.0
            px1, py1 = tl[0] + u*(tr[0]-tl[0]), tl[1] + u*(tr[1]-tl[1])
            px2, py2 = bl[0] + u*(br[0]-bl[0]), bl[1] + u*(br[1]-bl[1])
            cx, cy = px1 + v*(px2-px1), py1 + v*(py2-py1)
            row.append((int(cx - 6), int(cy - 6), 12, 12))
        rois.append(row)
    return rois

def cell_match_pixels(img, roi, thr):
    # 统计一个格子 ROI 内“落在该颜色阈值里的像素数量”。
    # 相比只看 ROI 的 LAB 均值，这种计数方式不会被格子边缘的背景蓝/彩色边框稀释：
    # 只要中心那团颜色像素够多就算命中，从根本上解决“黄色很清晰却时有时无”的问题。
    n = 0
    for b in img.find_blobs([thr], roi=roi, pixels_threshold=1, area_threshold=1, merge=True):
        n += b.pixels()
    return n

def scan_map(img, rois):
    # 地图扫描按“每个格子里各颜色的命中像素数”来分类，取占比最高的一类。
    # 用像素计数代替 LAB 均值：黄色箱子中心即使只露一部分，只要清晰可见就能稳定被标记，
    # 不会再因为 ROI 里混进背景色把均值拉出阈值而漏标。
    map_bits = bytearray(24)
    boxes, targets, bombs = [], [], []

    # 预设外墙
    for i in range(16):
        for j in range(12):
            if i==0 or i==15 or j==0 or j==11:
                idx = i*12 + j
                map_bits[idx // 8] |= (1 << (idx % 8))

    for r in range(10):
        for c in range(14):
            roi = rois[r][c]
            mx, my = r + 1, c + 1
            idx = my * 12 + mx

            # 分别统计四类颜色在该格子里的命中像素数。
            n_bomb = cell_match_pixels(img, roi, T_BOMB)
            n_tgt  = cell_match_pixels(img, roi, T_TGT)
            n_box  = cell_match_pixels(img, roi, T_BOX)
            n_wall = cell_match_pixels(img, roi, T_WALL)

            best = max(n_bomb, n_tgt, n_box, n_wall)
            if best < CELL_MIN_MATCH_PIXELS:
                continue  # 命中太少视为空地/背景，跳过

            # --- 颜色识别优先级逻辑（取命中像素最多的一类；同数时炸弹>目标>箱子>墙壁）---
            # 红色炸弹与粉紫目标的 a 轴范围重叠，但 b 轴天然分开，像素计数下二者不会互相抢格。
            if n_bomb == best:
                bombs.append((mx, my))
            elif n_tgt == best:
                targets.append((mx, my))
            elif n_box == best:
                boxes.append((mx, my))
            else:
                map_bits[idx//8] |= (1<<(idx%8))

    return map_bits, boxes, targets, bombs

def print_ascii_map(map_bits, boxes, targets, bombs, car_x, car_y):
    print("\n" + "="*38)
    print("======== 地图预览 [12x16] ========")
    gx, gy = int(car_x / GRID_SIZE_CM), int(car_y / GRID_SIZE_CM)
    for y in range(16):
        row_str = ""
        for x in range(12):
            idx = y * 12 + x
            is_wall = (map_bits[idx // 8] & (1 << (idx % 8))) != 0
            if x == gx and y == gy: row_str += " @ "
            elif (x, y) in bombs: row_str += " * "
            elif (x, y) in targets: row_str += " . "
            elif (x, y) in boxes: row_str += " $ "
            elif is_wall: row_str += " # "
            else: row_str += " - "
        print(row_str)
    print("="*38 + "\n")

def apply_illumination_correction(img):
    # 仅保留 OpenMV 原生自适应直方图均衡 (CLAHE)，让画面中心和边缘亮度更均匀。
    try:
        img.histeq(adaptive=True, clip_limit=CLAHE_CLIP_LIMIT)
    except Exception:
        pass
    return img

def capture_processed_frame(extra_frames=0):
    # extra_frames 用来丢掉刚切换模式后的过渡帧；最后统一做 gamma/亮度校正再交给扫描和跟踪。
    img = None
    for i in range(extra_frames + 1):
        img = sensor.snapshot()
    apply_illumination_correction(img)
    if ENABLE_GAMMA_CORR:
        try:
            img.gamma_corr(gamma=BASE_GAMMA, contrast=BASE_CONTRAST, brightness=BASE_BRIGHTNESS)
        except Exception:
            pass
    return img

def map_signature(map_bits, boxes, targets, bombs):
    # 稳定性判定用的“地图指纹”：只要位图或任意物体坐标变化，签名就会不同。
    sig = bytearray(map_bits)
    sig.extend(bytearray([len(boxes) & 0xFF, len(targets) & 0xFF, len(bombs) & 0xFF]))
    for p in boxes:
        sig.append((p[0] << 4) | p[1])
    for p in targets:
        sig.append((p[0] << 4) | p[1])
    for p in bombs:
        sig.append((p[0] << 4) | p[1])
    return sig

def maps_match(a, b):
    # 单独封装比较逻辑，后面如果要改成容错匹配，可以只改这里。
    return a == b

def cache_map_result(map_bits, boxes, targets, bombs):
    # 一旦拿到稳定完整地图，就把结果缓存起来，后面直接复用。
    global m_bits, m_boxes, m_targets, m_bombs, map_cache_ready
    m_bits = bytearray(map_bits)
    m_boxes = list(boxes)
    m_targets = list(targets)
    m_bombs = list(bombs)
    map_cache_ready = True

# ===========================================================================
# [6] 主程序
# ===========================================================================
init_camera()
clock = time.clock()
tracker = UltimateTracker(FIXED_CORNERS)
inner_rois = generate_inner_rois(FIXED_CORNERS)

m_bits, m_boxes, m_targets, m_bombs = bytearray(24), [], [], []
map_cache_ready = False
# 主控发地图请求后才采集/发送地图。
is_tracking_mode = False
map_capture_active = False
map_capture_skip = 0
map_stable_count = 0
map_scan_count = 0
map_prev_signature = None
map_send_repeat = 0
frame_count = 0

while True:
    clock.tick()
    frame_count += 1

    cmd = poll_uart_commands()
    if cmd == CMD_REQ_MAP:
        # 主控请求地图时，先暂停位姿发送。
        # 连续发车：每关地图都会变，收到请求就丢弃上一关缓存重新扫描，绝不复用旧地图。
        # 但采集进行中不要反复重启（会清零稳定计数，永远等不到稳定地图），
        # 所以只有当前不在扫描时才启动新一轮扫描。
        is_tracking_mode = False
        if not map_capture_active:
            map_cache_ready = False
            map_capture_active = True
            map_capture_skip = MAP_CAPTURE_SETTLE_FRAMES
            map_stable_count = 0
            map_scan_count = 0
            map_prev_signature = None
            map_send_repeat = 0
    elif cmd == CMD_REQ_POSE:
        is_tracking_mode = True
        if not map_capture_active and map_send_repeat == 0:
            send_pose_response((tracker.x, tracker.y, tracker.yaw))

    # 统一做一次图像亮度校正，让绿色阈值在中心和边缘都更稳定。
    # 这里的 gamma/brightness 已经按当前现场调好，除非整体环境光变化明显，否则不建议频繁改。
    img = capture_processed_frame()

    cmd = poll_uart_commands()
    if cmd == CMD_REQ_MAP:
        # 同上：连续发车时收到地图请求就重扫，不复用上一关缓存；采集进行中不重启。
        is_tracking_mode = False
        if not map_capture_active:
            map_cache_ready = False
            map_capture_active = True
            map_capture_skip = MAP_CAPTURE_SETTLE_FRAMES
            map_stable_count = 0
            map_scan_count = 0
            map_prev_signature = None
            map_send_repeat = 0
    elif cmd == CMD_REQ_POSE:
        is_tracking_mode = True

    if map_capture_active:
        if map_capture_skip > 0:
            map_capture_skip -= 1
        else:
            m_bits, m_boxes, m_targets, m_bombs = scan_map(img, inner_rois)
            map_scan_count += 1
            # 秒回策略：只要扫到任意非空地图（有箱/目标/炸弹之一）就立即打包发，
            # 不再苛求“箱子数==目标数”，避免为了凑齐条件白等好几帧。
            if len(m_boxes) > 0 or len(m_targets) > 0 or len(m_bombs) > 0:
                cache_map_result(m_bits, m_boxes, m_targets, m_bombs)
                map_send_repeat = MAP_SEND_REPEAT_COUNT
                map_capture_active = False
            # 连全空都没扫到就再试，最多扫 MAP_CAPTURE_MAX_SCAN_FRAMES 帧后兜底发出去。
            if map_capture_active and map_scan_count >= MAP_CAPTURE_MAX_SCAN_FRAMES:
                cache_map_result(m_bits, m_boxes, m_targets, m_bombs)
                map_send_repeat = MAP_SEND_REPEAT_COUNT
                map_capture_active = False
    elif map_send_repeat == 0:
        tracker.update(img)

    if map_send_repeat > 0:
        send_map_response(m_bits, m_boxes, m_targets, m_bombs)
        map_send_repeat -= 1
    elif is_tracking_mode:
        send_pose_response((tracker.x, tracker.y, tracker.yaw))

    if TRACK_DEBUG:
        # 跟踪调试图层：青色搜索 ROI、单层绿色小车框和坐标文字。
        if TRACK_DRAW_SEARCH_ROI:
            img.draw_rectangle(tracker.search_roi, color=(0, 180, 255))
        if TRACK_BLOB_DEBUG:
            for rect, reason in tracker.debug_blobs:
                img.draw_rectangle(rect, color=(120, 120, 120))
                img.draw_string(rect[0], rect[1] - 9, reason, color=(255, 255, 255), scale=1)
        if TRACK_DRAW_HEADING_ROIS:
            for roi in HEADING_ROIS:
                img.draw_rectangle(roi, color=(255, 0, 0))
        str_fps = "FPS:%.1f" % clock.fps()
        str_pose = "X:%.1f Y:%.1f A:%.1f" % (tracker.x, tracker.y, tracker.yaw)
        if tracker.debug_best_blob:
            img.draw_rectangle(tracker.debug_best_blob, color=(255, 255, 255))

        # 在图像左上角显示实时帧率和当前输出坐标。
        img.draw_string(11, 6, str_fps, color=(0, 0, 0), scale=1)
        img.draw_string(11, 26, str_pose, color=(0, 0, 0), scale=1)
        img.draw_string(10, 5, str_fps, color=(255, 255, 0), scale=1)
        img.draw_string(10, 25, str_pose, color=(255, 255, 0), scale=1)
        if ILLUMINATION_DRAW_LABEL:
            illum_text = "ILLUM:CLAHE"
            img.draw_string(11, 219, illum_text, color=(0, 0, 0), scale=1)
            img.draw_string(10, 218, illum_text, color=(255, 255, 0), scale=1)

    if DEBUG_MODE and (not is_tracking_mode) and frame_count % 15 == 0:
        # 地图扫描调试不需要每帧打印；15 帧打印一次，主要帧率留给小车跟踪。
        if not map_cache_ready:
            m_bits, m_boxes, m_targets, m_bombs = scan_map(img, inner_rois)
        if DEBUG_MODE:
            print_ascii_map(m_bits, m_boxes, m_targets, m_bombs, tracker.x, tracker.y)

    # 地图调试图层：白线外框、每个格子的采样点，以及识别出的箱子/目标/炸弹。
    if DEBUG_MODE:
        for i in range(4): # 画边界
            img.draw_line(FIXED_CORNERS[i][0], FIXED_CORNERS[i][1],
                          FIXED_CORNERS[(i+1)%4][0], FIXED_CORNERS[(i+1)%4][1], color=(255,255,255))

        for r in range(10):
            for c in range(14):
                roi = inner_rois[r][c]
                mx, my = r + 1, c + 1
                idx = my * 12 + mx
                is_wall = (m_bits[idx // 8] & (1 << (idx % 8))) != 0

                if (mx, my) in m_bombs:
                    img.draw_rectangle(roi, color=(0, 255, 0), fill=True)  # 炸弹上画绿色块
                elif (mx, my) in m_targets:
                    img.draw_rectangle(roi, color=(0, 0, 0), fill=True)    # 目标上画黑色块
                elif (mx, my) in m_boxes:
                    img.draw_rectangle(roi, color=(0, 0, 255), fill=True)  # 箱子上画蓝色块
                elif is_wall:
                    img.draw_rectangle(roi, color=(255, 0, 0), fill=True)  # 墙壁上画红色块
                else:
                    img.draw_circle(roi[0]+6, roi[1]+6, 1, color=(100,100,100))

        img.draw_string(10, 5, "FPS:%.1f" % clock.fps(), color=(0,255,0))
