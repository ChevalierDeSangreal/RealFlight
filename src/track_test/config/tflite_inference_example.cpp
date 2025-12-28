/**
 * TFLite Model Inference Example for TrackVer6 Policy
 * 
 * 这个示例展示如何在C++中使用TensorFlow Lite加载和推理TrackVer6策略模型
 * 适用于嵌入式设备部署
 * 
 * 编译命令示例:
 * g++ -std=c++17 tflite_inference_example.cpp \
 *     -I/path/to/tensorflow/lite/include \
 *     -L/path/to/tensorflow/lite/lib \
 *     -ltensorflow-lite \
 *     -o tflite_inference_example
 * 
 * 或使用CMake:
 * find_package(TensorFlowLite REQUIRED)
 * add_executable(tflite_inference_example tflite_inference_example.cpp)
 * target_link_libraries(tflite_inference_example TensorFlowLite::tensorflowlite)
 */

#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <deque>

// TensorFlow Lite头文件
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"

// 常量定义
constexpr int BUFFER_SIZE = 10;        // 动作-状态缓冲区大小
constexpr int ACTION_DIM = 4;          // 动作维度（四旋翼控制指令）
constexpr int OBS_DIM = 18;            // 观测维度（根据TrackEnvVer6）
constexpr int INPUT_DIM = BUFFER_SIZE * (OBS_DIM + ACTION_DIM);  // 总输入维度
constexpr int ACTION_REPEAT = 10;      // 动作重复次数

/**
 * 动作-状态缓冲区类
 * 维护最近BUFFER_SIZE个时间步的观测和动作
 */
class ActionObsBuffer {
public:
    ActionObsBuffer(int buffer_size, int obs_dim, int action_dim)
        : buffer_size_(buffer_size), obs_dim_(obs_dim), action_dim_(action_dim) {
        // 初始化缓冲区为全零
        for (int i = 0; i < buffer_size_; ++i) {
            obs_buffer_.push_back(std::vector<float>(obs_dim_, 0.0f));
            action_buffer_.push_back(std::vector<float>(action_dim_, 0.0f));
        }
    }
    
    // 更新缓冲区（添加新的观测和动作）
    void update(const std::vector<float>& obs, const std::vector<float>& action) {
        // 移除最旧的数据
        obs_buffer_.pop_front();
        action_buffer_.pop_front();
        
        // 添加最新的数据
        obs_buffer_.push_back(obs);
        action_buffer_.push_back(action);
    }
    
    // 获取展平的缓冲区数据作为网络输入
    std::vector<float> get_flattened_buffer() const {
        std::vector<float> flattened;
        flattened.reserve(buffer_size_ * (obs_dim_ + action_dim_));
        
        for (int i = 0; i < buffer_size_; ++i) {
            // 添加观测
            flattened.insert(flattened.end(), obs_buffer_[i].begin(), obs_buffer_[i].end());
            // 添加动作
            flattened.insert(flattened.end(), action_buffer_[i].begin(), action_buffer_[i].end());
        }
        
        return flattened;
    }
    
    // 重置缓冲区
    void reset() {
        for (auto& obs : obs_buffer_) {
            std::fill(obs.begin(), obs.end(), 0.0f);
        }
        for (auto& action : action_buffer_) {
            std::fill(action.begin(), action.end(), 0.0f);
        }
    }

private:
    int buffer_size_;
    int obs_dim_;
    int action_dim_;
    std::deque<std::vector<float>> obs_buffer_;
    std::deque<std::vector<float>> action_buffer_;
};

/**
 * TFLite策略推理器类
 */
class TFLitePolicyInference {
public:
    TFLitePolicyInference(const std::string& model_path)
        : buffer_(BUFFER_SIZE, OBS_DIM, ACTION_DIM), 
          step_counter_(0),
          last_action_(ACTION_DIM, 0.0f) {
        
        // 加载模型
        model_ = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
        if (!model_) {
            std::cerr << "Failed to load model from: " << model_path << std::endl;
            return;
        }
        
        // 创建解释器
        tflite::ops::builtin::BuiltinOpResolver resolver;
        tflite::InterpreterBuilder builder(*model_, resolver);
        builder(&interpreter_);
        
        if (!interpreter_) {
            std::cerr << "Failed to create interpreter" << std::endl;
            return;
        }
        
        // 分配张量
        if (interpreter_->AllocateTensors() != kTfLiteOk) {
            std::cerr << "Failed to allocate tensors" << std::endl;
            return;
        }
        
        // 打印模型信息
        print_model_info();
        
        std::cout << "✅ TFLite model loaded successfully!" << std::endl;
    }
    
    /**
     * 获取动作（考虑动作重复）
     * @param obs 当前观测
     * @return 控制动作
     */
    std::vector<float> get_action(const std::vector<float>& obs) {
        // 每ACTION_REPEAT步才计算新动作
        if (step_counter_ % ACTION_REPEAT == 0) {
            last_action_ = compute_action(obs);
        }
        
        // 更新缓冲区
        buffer_.update(obs, last_action_);
        step_counter_++;
        
        return last_action_;
    }
    
    /**
     * 重置推理器状态
     */
    void reset() {
        buffer_.reset();
        step_counter_ = 0;
        std::fill(last_action_.begin(), last_action_.end(), 0.0f);
    }
    
    /**
     * 性能基准测试
     * @param num_iterations 测试迭代次数
     */
    void benchmark(int num_iterations = 1000) {
        std::cout << "\n开始性能基准测试 (" << num_iterations << " 次推理)..." << std::endl;
        
        // 准备随机输入
        std::vector<float> random_obs(OBS_DIM);
        for (int i = 0; i < OBS_DIM; ++i) {
            random_obs[i] = static_cast<float>(rand()) / RAND_MAX * 2.0f - 1.0f;
        }
        
        // 预热
        for (int i = 0; i < 10; ++i) {
            compute_action(random_obs);
        }
        
        // 计时
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < num_iterations; ++i) {
            compute_action(random_obs);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        double avg_time_us = static_cast<double>(duration.count()) / num_iterations;
        double avg_time_ms = avg_time_us / 1000.0;
        double fps = 1000000.0 / avg_time_us;
        
        std::cout << "  总时间: " << duration.count() / 1000.0 << " ms" << std::endl;
        std::cout << "  平均推理时间: " << avg_time_us << " μs (" << avg_time_ms << " ms)" << std::endl;
        std::cout << "  推理频率: " << fps << " Hz" << std::endl;
    }

private:
    /**
     * 计算动作（实际推理）
     * @param obs 当前观测
     * @return 控制动作
     */
    std::vector<float> compute_action(const std::vector<float>& obs) {
        // 获取展平的缓冲区数据
        std::vector<float> input_data = buffer_.get_flattened_buffer();
        
        // 获取输入张量
        float* input = interpreter_->typed_input_tensor<float>(0);
        
        // 复制输入数据
        std::copy(input_data.begin(), input_data.end(), input);
        
        // 推理
        if (interpreter_->Invoke() != kTfLiteOk) {
            std::cerr << "Failed to invoke interpreter" << std::endl;
            return std::vector<float>(ACTION_DIM, 0.0f);
        }
        
        // 获取输出
        float* output = interpreter_->typed_output_tensor<float>(0);
        
        // 复制输出数据
        std::vector<float> action(output, output + ACTION_DIM);
        
        return action;
    }
    
    /**
     * 打印模型信息
     */
    void print_model_info() {
        std::cout << "\n模型信息:" << std::endl;
        
        // 输入信息
        const auto* input_tensor = interpreter_->input_tensor(0);
        std::cout << "  输入张量:" << std::endl;
        std::cout << "    名称: " << input_tensor->name << std::endl;
        std::cout << "    维度: [";
        for (int i = 0; i < input_tensor->dims->size; ++i) {
            std::cout << input_tensor->dims->data[i];
            if (i < input_tensor->dims->size - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        std::cout << "    类型: " << TfLiteTypeGetName(input_tensor->type) << std::endl;
        
        // 输出信息
        const auto* output_tensor = interpreter_->output_tensor(0);
        std::cout << "  输出张量:" << std::endl;
        std::cout << "    名称: " << output_tensor->name << std::endl;
        std::cout << "    维度: [";
        for (int i = 0; i < output_tensor->dims->size; ++i) {
            std::cout << output_tensor->dims->data[i];
            if (i < output_tensor->dims->size - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        std::cout << "    类型: " << TfLiteTypeGetName(output_tensor->type) << std::endl;
        
        std::cout << std::endl;
    }

private:
    std::unique_ptr<tflite::FlatBufferModel> model_;
    std::unique_ptr<tflite::Interpreter> interpreter_;
    ActionObsBuffer buffer_;
    int step_counter_;
    std::vector<float> last_action_;
};

/**
 * 主函数 - 使用示例
 */
int main(int argc, char* argv[]) {
    std::cout << "===========================================================" << std::endl;
    std::cout << "TFLite TrackVer6 Policy Inference Example" << std::endl;
    std::cout << "===========================================================" << std::endl;
    
    // 模型文件路径
    std::string model_path = "../param/tflite/trackVer6_policy.tflite";
    if (argc > 1) {
        model_path = argv[1];
    }
    
    std::cout << "加载模型: " << model_path << std::endl;
    
    // 创建推理器
    TFLitePolicyInference policy(model_path);
    
    // ==================== 示例1: 单次推理 ====================
    std::cout << "\n示例1: 单次推理" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    
    // 模拟观测数据（实际使用时应该来自传感器）
    std::vector<float> obs(OBS_DIM);
    for (int i = 0; i < OBS_DIM; ++i) {
        obs[i] = static_cast<float>(rand()) / RAND_MAX * 2.0f - 1.0f;
    }
    
    std::cout << "输入观测 (前5个): [";
    for (int i = 0; i < std::min(5, OBS_DIM); ++i) {
        std::cout << obs[i];
        if (i < std::min(5, OBS_DIM) - 1) std::cout << ", ";
    }
    std::cout << " ...]" << std::endl;
    
    // 获取动作
    std::vector<float> action = policy.get_action(obs);
    
    std::cout << "输出动作: [";
    for (int i = 0; i < ACTION_DIM; ++i) {
        std::cout << action[i];
        if (i < ACTION_DIM - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    // ==================== 示例2: 模拟控制循环 ====================
    std::cout << "\n示例2: 模拟控制循环 (100步)" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    
    policy.reset();
    
    for (int step = 0; step < 100; ++step) {
        // 模拟观测更新
        for (int i = 0; i < OBS_DIM; ++i) {
            obs[i] += (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.1f;
            obs[i] = std::max(-1.0f, std::min(1.0f, obs[i]));  // 裁剪到[-1, 1]
        }
        
        // 获取动作
        action = policy.get_action(obs);
        
        // 每10步打印一次
        if (step % 10 == 0) {
            std::cout << "  步骤 " << step << " - 动作: [";
            for (int i = 0; i < ACTION_DIM; ++i) {
                std::cout << action[i];
                if (i < ACTION_DIM - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
    }
    
    // ==================== 示例3: 性能基准测试 ====================
    policy.benchmark(1000);
    
    std::cout << "\n===========================================================" << std::endl;
    std::cout << "示例完成！" << std::endl;
    std::cout << "===========================================================" << std::endl;
    
    return 0;
}

