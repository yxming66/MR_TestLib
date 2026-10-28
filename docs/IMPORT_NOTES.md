# MRobot 测试版本

整理日期：2026-10-26。

## 目录

- `User/`：当前测试代码，共 283 个文件。以 RC_02 的当前磁盘内容为基础，再合入 zzrobot 的 10 个差异文件。保留原有 C/C++ 目录结构及 YAML 配置，不改写算法。
- `_reference/User_code/`：旧版可信代码完整快照，共 156 个文件（含隐藏辅助文件）。旧版特有的驱动、模板和 CSV 元数据保留在此，不自动加入当前测试代码编译。
- `SOURCE_MANIFEST.csv`：逐文件记录当前版本来源，以及结果、RC_02、zzrobot 的 SHA-256。
- `BASELINE_MANIFEST.csv`：旧版快照逐文件 SHA-256。
- `build/host_tests/`：运行已有机械臂主机测试生成的产物。

## 来源与选择规则

旧版可信基线：`D:\apps\MRobot\MRobot\assets\User_code`

后续开发主体：`D:\STM32Projects_HAL\board-H7\RC_02\User`

机械臂后续分支：`D:\STM32Projects_HAL\board-H7\zzrobot\User`

RC_02 和 zzrobot 各有 283 个文件，路径集合完全相同，273 个文件内容完全相同。仅以下 10 个文件不同，全部采用 zzrobot 内容：

- `module/arm/arm_command_adapter.cpp`
- `module/arm/arm_model_config.cpp`
- `robotics/arm/control/arm_controller.hpp`
- `robotics/arm/core/command.hpp`
- `robotics/arm/core/status.hpp`
- `robotics/arm/kinematics/inverse_kinematics.hpp`
- `robotics/arm/README.md`
- `robotics/arm/tests/arm_host_tests.cpp`
- `robotics/core/rotation.cpp`
- `robotics/core/rotation.hpp`

最后两个文件是机械臂算法使用的共享旋转数学依赖，因此一并同步。当前 User 内容与 zzrobot/User 完全一致；分文件来源记录仍区分 RC_02 主体与 zzrobot 更新。

旧版采用按库分子目录的布局，RC_02 已将大量 C 文件移到各层根目录，并新增 C++ 库。为避免重复实现和旧配置混入编译，没有把旧目录直接叠加到 User。需要旧版独有驱动时，可从参考快照单独迁移。参考快照是历史可信版本；新增内容仅表示已同步并通过下述检查，不表示全部完成硬件验证。

## 验证

- 当前 283 个文件均通过 SHA-256 核对。
- 旧版快照 156 个文件均通过 SHA-256 核对。
- 已运行原有 `User/robotics/arm/tests/run_host_tests.ps1`，结果：`arm host tests passed`。
- 未修改三个源目录。
- 本目录是 User 代码测试集合，不是完整 STM32 工程；未复制 Core、Drivers、Middlewares、链接脚本和板级构建配置，未进行完整固件编译、烧录或实机测试。

在安装了 g++ 的 PowerShell 中，于本目录执行以下命令可重跑已有主机测试：

```powershell
.\User\robotics\arm\tests\run_host_tests.ps1
```
