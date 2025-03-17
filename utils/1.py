import matplotlib.pyplot as plt
import matplotlib.patches as patches

# 使用支持中文的字体（如SimHei），若环境中无此字体，请安装或更换为其他中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(16, 10))
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.axis('off')

# 主标题
ax.text(50, 97, "研究路径图：基于智能感知与AR的人机协作线缆装配系统", ha='center', va='center', fontsize=16, fontweight='bold')

# 顶部横幅
rect_main = patches.Rectangle((5, 90), 90, 5, linewidth=1, edgecolor='black', facecolor='#E0E0E0')
ax.add_patch(rect_main)
ax.text(50, 92.5, "工业5.0背景下的线缆装配人机协作框架", ha='center', va='center', fontsize=12)

# 定义上表格区域及各列范围
columns = [ (5, 27.5), (27.5, 50), (50, 72.5), (72.5, 95) ]
layer_names = ["智能感知层", "决策控制层", "AR交互层", "系统集成层"]

# 绘制上部区域边框 (从y=45到y=70)
rect_upper = patches.Rectangle((5,45), 90, 25, linewidth=1, edgecolor='black', facecolor='none')
ax.add_patch(rect_upper)
# 分隔线 (水平分割：y=60)
ax.plot([5,95],[60,60], color='black')

# 绘制各列分隔线及列标题
for i, (x1, x2) in enumerate(columns):
    cx = (x1+x2)/2
    ax.text(cx, 67.5, layer_names[i], ha='center', va='center', fontsize=12, fontweight='bold')
    if i < len(columns)-1:
        ax.plot([x2, x2], [45, 70], color='black')

# 上半部分内容 (y:60~70)
# 智能感知层
col1_x = (5+27.5)/2
texts_col1 = [
    "■ 多模态感知",
    "  - YOLOv11目标检测",
    "  - D435i深度感知",
    "  - 点云配准"
]
start_y = 65
for i, line in enumerate(texts_col1):
    ax.text(col1_x, start_y - i*2.5, line, ha='center', va='center', fontsize=10)

# 决策控制层
col2_x = (27.5+50)/2
texts_col2 = [
    "■ 动态路径规划",
    "  - 避障算法",
    "  - 力控策略",
    "",
    "■ 运动学仿真",
    "  - Unity仿真平台"
]
start_y = 65
for i, line in enumerate(texts_col2):
    ax.text(col2_x, start_y - i*2.5, line, ha='center', va='center', fontsize=10)

# AR交互层
col3_x = (50+72.5)/2
texts_col3 = [
    "■ AR空间映射",
    "  - HoloLens2",
    "  - 环境建模",
    "",
    "■ 虚实融合引导",
    "  - 装配导航"
]
start_y = 65
for i, line in enumerate(texts_col3):
    ax.text(col3_x, start_y - i*2.5, line, ha='center', va='center', fontsize=10)

# 系统集成层
col4_x = (72.5+95)/2
texts_col4 = [
    "■ 多设备协同",
    "  - TCP/IP通信",
    "  - 数据中台",
    "",
    "■ 安全验证",
    "  - 可靠性测试"
]
start_y = 65
for i, line in enumerate(texts_col4):
    ax.text(col4_x, start_y - i*2.5, line, ha='center', va='center', fontsize=10)

# 下半部分内容 (y:45~60)
# 智能感知层的关键技术突破
ax.text(col1_x, 57, "关键技术突破", ha='center', va='center', fontsize=10, fontweight='bold')
texts_lower_col1 = [
    "- 识别误差<0.5mm",
    "- 抗环境干扰算法"
]
for i, line in enumerate(texts_lower_col1):
    ax.text(col1_x, 54 - i*3, line, ha='center', va='center', fontsize=10)

# 决策控制层的关键技术突破
ax.text(col2_x, 57, "关键技术突破", ha='center', va='center', fontsize=10, fontweight='bold')
texts_lower_col2 = [
    "- 路径优化30%",
    "- 动态避障响应<1s"
]
for i, line in enumerate(texts_lower_col2):
    ax.text(col2_x, 54 - i*3, line, ha='center', va='center', fontsize=10)

# AR交互层的关键技术突破
ax.text(col3_x, 57, "关键技术突破", ha='center', va='center', fontsize=10, fontweight='bold')
texts_lower_col3 = [
    "- 实时投影延迟<50ms",
    "- 多模态交互"
]
for i, line in enumerate(texts_lower_col3):
    ax.text(col3_x, 54 - i*3, line, ha='center', va='center', fontsize=10)

# 系统集成层的系统特性
ax.text(col4_x, 57, "系统特性", ha='center', va='center', fontsize=10, fontweight='bold')
texts_lower_col4 = [
    "- 模块化设计",
    "- 支持跨场景迁移"
]
for i, line in enumerate(texts_lower_col4):
    ax.text(col4_x, 54 - i*3, line, ha='center', va='center', fontsize=10)

# 绘制指向下方区域的箭头
ax.annotate("", xy=(50, 45), xytext=(50, 40), arrowprops=dict(arrowstyle="->", lw=2))

# 下方区域：两个并列框
rect_lower = patches.Rectangle((5,20), 90, 20, linewidth=1, edgecolor='black', facecolor='none')
ax.add_patch(rect_lower)
# 分隔线
ax.plot([50,50],[20,40], color='black')

# 左侧：典型应用场景
ax.text((5+50)/2, 38, "典型应用场景", ha='center', va='center', fontsize=10, fontweight='bold')
texts_lower_left = [
    "■ 连接器精准装配",
    "■ 复杂线束布局",
    "■ 高危工序协作",
    "■ 新手操作培训"
]
for i, line in enumerate(texts_lower_left):
    ax.text((5+50)/2, 33 - i*3, line, ha='center', va='center', fontsize=10)

# 右侧：技术优势对比
ax.text((50+95)/2, 38, "技术优势对比", ha='center', va='center', fontsize=10, fontweight='bold')
texts_lower_right = [
    "➤ 定位精度提升40%",
    "➤ 装配效率提高35%",
    "➤ 工伤风险降低60%",
    "➤ 培训周期缩短50%"
]
for i, line in enumerate(texts_lower_right):
    ax.text((50+95)/2, 33 - i*3, line, ha='center', va='center', fontsize=10)

# 保存为JPG图片
plt.savefig("research_pathway.jpg", format="jpg", dpi=300, bbox_inches='tight')
plt.show()
