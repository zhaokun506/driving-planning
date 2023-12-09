## 1.概述
    config-参数配置的类，将参数配置为类
    EMPlanner-主算法入口

    path_time是构建SL图。
        1.S坐标值的建立
        2.host在SL图的坐标
        3.障碍物在SL图的坐标
        4.采样点的构造
        5.动态规划
        6.二次规划

    speed_time是构建ST图。
        1.S坐标值的建立
        2.ST图的建立
        3.采样点的构造
        4.动态规划
        5.二次规划

    reference_line参考线模块
        1.参考线的截取
        2.参考线的平滑

## 2.使用方法
1.ubuntu20.04 自行安装python cmake库
2.安装第三方依赖库(我已经安装完毕，在bin/libary目录下)
     eigen3.3.7
     osqp0.6.3
     osqp-eigen0.8.0
3. source tools/BuildEnv
4. 修改 src/CMakeLists.txt内的python路径
5. make build
6. cd build
7. cmake ../src
8. make
9. ./bin/path_plan_test




## 3.结果展示     
参考线平滑结果：
![参考系平滑](https://user-images.githubusercontent.com/54465004/201812152-533bb555-c0a2-46b2-8369-432a21c60b68.png)

路径规划结果dp+qp   SL图
![Figure_3](https://user-images.githubusercontent.com/54465004/204950208-a4ef9fae-c9d6-49b1-ab59-05f498b4aa27.png)




速度规划结果dp+qp  ST图

![Figure_3](https://user-images.githubusercontent.com/54465004/204949997-f21350b4-5d23-40d7-a326-97e12a8095c8.png)


## 4.参考文献：
- 参照b站忠厚老实的老王 自动驾驶决策规划算法
- 百度apollo
- 湖南大学 李柏 自动驾驶决策规划算法

## 5.声明及注意
    1.水平有限，不喜勿喷，仅供学习使用。引用请声明
    2. 由于程序最后一个版本丢失，效果和图片展示的效果不太一样，需要自行调试。
    3.routing perception localization做了理想假设
    4.程序是半成品，只有一个周期，有待调试debug。后面我会优化更新
    5.程序写的比较烂，完全不是面向对象的，后面会优化结构，但是流程比较清晰，可以参考学习。
