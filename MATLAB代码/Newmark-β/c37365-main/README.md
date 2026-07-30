# 刚体动力学仿真示例程序

## 简介
本仓库提供了一阶旋转倒立摆系统动力学的MATLAB仿真程序，涵盖了三种不同的仿真方法：`ode45`、`Simulink` 及 `Simscape Multibody`。通过这些示例程序，您可以深入了解刚体动力学方程的求解与仿真过程。

## 资源内容
- **ode45方法**：使用MATLAB内置的`ode45`函数进行数值求解。
- **Simulink方法**：利用Simulink搭建仿真模型，直观展示系统动力学行为。
- **Simscape Multibody方法**：通过Simscape Multibody模块进行多体动力学仿真。

## 使用说明
1. **下载资源**：点击仓库页面右上角的“Code”按钮，选择“Download ZIP”下载整个仓库的压缩包，或使用`git clone`命令克隆仓库到本地。
2. **运行仿真**：
   - 对于`ode45`方法，直接运行对应的MATLAB脚本文件。
   - 对于`Simulink`方法，打开Simulink模型文件并运行仿真。
   - 对于`Simscape Multibody`方法，打开对应的Simulink模型文件并运行仿真。

## 参考资料
更多详细内容及理论背景，请参阅博客文章：[从刚体动力学方程到 MATLAB 多种方法仿真验证](https://blog.csdn.net/qq_41658212/article/details/118799511)。

## 贡献
欢迎提交问题、建议或改进代码的Pull Request。您的贡献将帮助更多人学习和使用这些仿真方法。

## 许可证
本项目采用MIT许可证，详情请参阅[LICENSE](LICENSE)文件。