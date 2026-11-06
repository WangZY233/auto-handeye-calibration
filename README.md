# 可扩展手眼标定 / Extensible hand-eye calibration

本项目从 NERO 双臂标定代码整理，提供与品牌无关的 **眼在手外（eye-to-hand）**离线求解和手动采集接口。机械臂品牌、自由度和 SDK 不进入求解器；其他设备需要自行实现适配器，项目尚未对所有机械臂或相机完成实机验证。眼在手上、自动运动规划、碰撞检测和 TCP 接触验证不属于这个通用入口。

## 安装与使用

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .
handeye solve examples/synthetic.json --output result.json
```

`result.json` 的 `T_base_camera` 将相机坐标转换到机械臂基座，`T_flange_board` 将标定板坐标转换到反馈末端。平移单位米。输出中包含拟合残差、共识内点及独立验证点残差；残差小不代表物理精度已经验证。示例仅为合成数据。

## 如何接入不同机械臂？

接入新机械臂需要实现一个 Python 适配器。项目不依赖特定品牌的机械臂 SDK，也不自带通用驱动；请先安装机械臂和相机各自的 SDK、驱动，并确认能读取机械臂实际反馈和相机图像。以 [适配器模板](examples/adapter_template.py) 为起点：

1. **创建适配器模块。** 复制模板为 `my_adapter.py`，保留 `create()` 工厂函数；在工厂函数中连接设备并返回适配器实例。模块须能通过 Python 导入。
2. **接入录制轨迹。** 在 `execute_segment(segment)` 中将每段 `waypoints` 转换成该机械臂 SDK 接受的关节顺序、单位、时间和控制命令，整段发送；根据控制器实际反馈等待到位和稳定后返回。异常、超时或未到位时应抛出异常。返回值应为可 JSON 序列化的执行记录（也可为 `None`）。轨迹应按采样位置分段，具体格式见下方“使用已录制轨迹采集”。
3. **接入机器人反馈和相机。** 在 `read_pair()` 中返回同步的 `T_base_flange`、`T_camera_board` 和可选的 `metadata`。前者是实际控制器反馈末端到机器人基座的变换，后者是标定板到相机光学坐标系的变换；两者均为平移单位为米的 4×4 齐次矩阵。标定板识别、相机内参及设备时间同步由适配器实现；不要用运动目标代替实际反馈。建议在 `metadata` 中记录设备、标定板和两侧时间戳。
4. **释放设备。** 在 `close()` 中断开机械臂、相机及其他资源。执行和采集异常时程序也会调用该方法。
5. **运行并检查结果。** 把适配器放到 Python 搜索路径，将已录制轨迹转换为本项目的 JSON 外壳后执行：

   ```bash
   PYTHONPATH=examples handeye replay my_trajectory.json --adapter my_adapter:create --output poses.json
   handeye solve poses.json --output result.json
   ```

   路径 `examples` 应替换为 `my_adapter.py` 所在目录；如果它已在 Python 搜索路径中，可省略 `PYTHONPATH`。回放会调用适配器控制机械臂；请按实际设备配置轨迹及执行参数。项目本身不提供运动规划和碰撞检测，也不会自动适配厂商轨迹格式。完成后应检查采集状态、反馈与拟合残差，并进行独立物理精度验证。

也可以先不接入运动控制，仅实现 `read_pair()` 和 `close()`，由操作者手动移动并停稳机械臂后采集：

```bash
PYTHONPATH=examples handeye collect --adapter my_adapter:create --count 20 --output poses.json
handeye solve poses.json --output result.json
```

所有矩阵为 4×4 齐次变换，`T_A_B` 表示把 B 坐标中的点变换到 A；旋转为右手坐标系正交旋转矩阵，位移为米。相机必须固定在基座附近，标定板刚性安装在反馈末端上；不要混用法兰、TCP 或旋转后的图像坐标系。拟合模型为 `T_base_flange @ T_flange_board = T_base_camera @ T_camera_board`。采集多个不同位置和旋转轴的姿态；避免仅平移或绕单轴转动。

也可直接准备 JSON：`{"samples": [{"T_base_flange": [[...]], "T_camera_board": [[...]], "role": "train"}]}`。将未参与拟合的样本标为 `validation`；默认 `train`。建议额外进行独立物理测量并评估误差。验证集若用于挑选算法或参数，就不再是独立测试集。

## 开源准备

本项目采用仓库中的 [MIT 许可证](LICENSE)。标定求解算法从原项目中提取，设备驱动与历史数据未包含；使用第三方依赖时请遵守其各自的许可证。

测试：`python -m unittest discover -s tests`。

## 使用已录制轨迹采集

`replay` 将录制轨迹按段发送给用户提供的机械臂适配器，并在标记的段末采集反馈位姿和相机标定位姿：

```bash
PYTHONPATH=examples handeye replay my_trajectory.json --adapter my_adapter:create --output poses.json
handeye solve poses.json --output result.json
```

轨迹 JSON 使用以下通用外壳，完整示例见 [模拟轨迹](examples/simulated_trajectory.json)：

```json
{
  "version": 1,
  "joint_names": ["joint1", "joint2"],
  "segments": [
    {
      "name": "first_capture",
      "waypoints": [
        {"time_from_start_s": 0.0, "joint_positions_rad": [0.0, 0.0]},
        {"time_from_start_s": 2.0, "joint_positions_rad": [0.2, -0.3]}
      ],
      "capture": true,
      "role": "train"
    }
  ]
}
```

`segments` 按录制顺序执行；每段 `waypoints` 原样交给适配器。示例使用关节弧度和段内相对秒数，用户也可在路点中使用适配器约定的其他表示；**本项目不自动转换厂商格式或解析时间、关节顺序**，应在适配器中完成转换，并按实际录制内容设置运动参数。`joint_names` 等顶层信息供归档；适配器若需要这些信息，应将其配置在适配器中或写入每个段。`capture: true` 表示该段终点采样，`false` 可用于只经过的过渡段；`role` 可设 `train` 或 `validation`。需要停稳采样的位置应作为段终点，段内完整轨迹一次交给适配器，避免逐路点停止。

适配器的 `execute_segment(segment)` 必须发送整段轨迹，并根据实际机器人反馈等待到位和稳定；失败时抛出异常。返回值必须能序列化为 JSON，可记录控制器反馈与执行信息。`read_pair()` 应返回该时刻同步的相机观测与**实际反馈**位姿；请在 `metadata` 记录时间戳及同步方式。执行和碰撞检查由用户的适配器及控制系统负责，本项目不提供通用规划或碰撞检测。

程序每段将轨迹、已完成段数、已采样数据和状态保存到输出文件；执行或采样失败时记录 `failed` 并停止，不自动续跑。检查原因后可根据已完成段数自行准备剩余轨迹，注意避免重复执行已完成的动作。输出文件不能已存在。轨迹记录的是目标，不能替代真实反馈。

不连接硬件的完整演示：

```bash
PYTHONPATH=examples handeye replay examples/simulated_trajectory.json --adapter simulated_adapter:create --output /tmp/poses-demo.json
handeye solve /tmp/poses-demo.json --output /tmp/result-demo.json
```

模拟适配器返回预先生成的合成观测，不代表实际轨迹执行或物理精度。
