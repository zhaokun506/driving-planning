
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
![动态规划](https://user-images.githubusercontent.com/54465004/203503595-d1f27155-b44d-4e37-9a41-67398ded8c28.png)

![路径二次规划](https://user-images.githubusercontent.com/54465004/203503604-3117dc5f-d96a-4b03-b6da-b9b32ed09d82.png)


参考文献：
参照b站忠厚老实的老王 自动驾驶决策规划算法，routing perception localization做了理想假设
apollo
湖南大学 李柏 自动驾驶决策规划算法

