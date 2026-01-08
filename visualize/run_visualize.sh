#!/bin/bash

# 飞行数据可视化运行脚本

# 检查参数
if [ $# -eq 0 ]; then
    echo "用法: $0 <bag文件路径>"
    echo ""
    echo "示例:"
    echo "  $0 ../fly_log/track_moving_05"
    echo "  $0 ../fly_log/track_moving_05_1"
    echo ""
    echo "注意: 请先 source ROS2 环境："
    echo "  source /opt/ros/humble/setup.bash"
    echo "  source ~/wangzimo/RealFlight/install/setup.bash"
    exit 1
fi

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 错误: 未检测到 ROS2 环境"
    echo "请先 source ROS2 环境："
    echo "  source /opt/ros/humble/setup.bash"
    echo "  source ~/wangzimo/RealFlight/install/setup.bash"
    exit 1
fi

echo "✓ ROS2 环境: $ROS_DISTRO"

BAG_PATH=$1

# 检查bag文件是否存在
if [ ! -d "$BAG_PATH" ]; then
    echo "错误: 找不到bag文件: $BAG_PATH"
    exit 1
fi

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 进入脚本目录
cd "$SCRIPT_DIR"

echo "=================================================="
echo "飞行数据可视化工具"
echo "=================================================="
echo "Bag路径: $BAG_PATH"
echo "=================================================="
echo ""

# 运行Python脚本
python3 visualize.py "$BAG_PATH"

# 检查运行结果
if [ $? -eq 0 ]; then
    echo ""
    echo "=================================================="
    echo "✅ 可视化完成！"
    echo "=================================================="
    
    # 获取输出目录
    BAG_NAME=$(basename "$BAG_PATH")
    OUTPUT_DIR="output/$BAG_NAME"
    
    echo "输出目录: $OUTPUT_DIR"
    echo ""
    echo "生成的文件:"
    ls -lh "$OUTPUT_DIR"
    echo "=================================================="
else
    echo ""
    echo "❌ 可视化失败，请检查错误信息"
    exit 1
fi

