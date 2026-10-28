#pragma once

// 观测器聚合接口。使用方式：包含本文件后构造具体观测器，配置模型和采样周期，
// 在控制周期调用 Update(measurement, input)，再读取估计状态。

// 观测器组件聚合头文件。
#include "component/observer/linear_observer.hpp"
