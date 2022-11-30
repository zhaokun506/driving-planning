
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
        
参考线平滑结果：
![参考系平滑](https://user-images.githubusercontent.com/54465004/201812152-533bb555-c0a2-46b2-8369-432a21c60b68.png)

路径规划结果dp+qp

![Figure_2](https://user-images.githubusercontent.com/54465004/204689819-51f67ca6-7850-4b67-97b5-471b003a742b.png)


速度规划结果dp

![Figure_3](https://user-images.githubusercontent.com/54465004/204689834-06f5209e-a6f0-4d22-88df-6a407d62d424.png)
![Figure_4](https://user-images.githubusercontent.com/54465004/204689838-2c046971-0773-4c92-9f0a-9ad26b76393b.png)




参考文献：
参照b站忠厚老实的老王 自动驾驶决策规划算法，routing perception localization做了理想假设

apollo

湖南大学 李柏 自动驾驶决策规划算法


