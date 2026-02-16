1️⃣ 最常见原因：QEF 解跑出 cell（90%概率）

如果你没有做 clamp：

vertex ∉ cell bounds


QEF 可能会解出：

远离交点的极端位置


尤其当：

平面接近平行

法向接近共线

rank deficiency

就会爆炸。

✅ 正确做法
if (!point_inside_cell(v))
    v = clamp_to_cell(v);


或者：

v = v.cwiseMax(cell_min).cwiseMin(cell_max);

🔥 2️⃣ QEF 退化（rank deficiency）

如果一个 cell 里：

所有法向几乎共线

或只有 1~2 个交点

矩阵 A 的 rank < 3

这时最小二乘问题没有唯一解。

结果会：

SVD 给出极端解

顶点飞出 cell

✅ 解决方法

判断奇异值：

if (smallest_singular_value < epsilon)
    fallback_to_cell_center();

🔥 3️⃣ 法向方向不一致

如果 Hermite normal：

有的指 inside

有的指 outside

那 QEF 平面约束会互相冲突。

效果就是：

顶点位置剧烈震荡

✅ 检查

确认：

normal = normalize(gradient);


而且方向统一：

比如 SDF > 0 是 outside。

🔥 4️⃣ 法向未归一化

QEF 构造的是：

𝑛
𝑖
⋅
𝑥
=
𝑛
𝑖
⋅
𝑝
𝑖
n
i
	​

⋅x=n
i
	​

⋅p
i
	​


如果 n_i 不归一化：

某些平面权重异常大

解被拉偏

🔥 5️⃣ 面连接顺序错误（较低概率）

如果 face indices 顺序错：

winding 错

出现翻折

但你这个更像 vertex 错。

🔥 6️⃣ Grid resolution 太低

壶嘴和壶柄是高曲率区域。

如果 grid 太粗：

Hermite sample 少

QEF 不稳定

但这通常不会产生尖刺这么严重。

🔬 你这个图的直觉判断

从“刺状拉伸”来看：

99% 是 QEF 解没有被限制在 cell 内

这是 DC 初学者最常见 bug。

🎯 你可以立刻做的测试
在 solve 后打印：
if (!point_inside_cell(v)) {
    std::cout << "QEF outside cell!" << std::endl;
}


我敢说你会看到一堆输出。

🧠 为什么 DC 容易这样？

因为 QEF 本质是：

min
⁡
∣
∣
𝐴
𝑥
−
𝑏
∣
∣
2
min∣∣Ax−b∣∣
2

这是无约束最小二乘。

但 DC 需要的是：

约束最小二乘（解必须在 cell 内）

而原论文默认没有强约束。

所以工程实现必须补 clamp。

🚀 推荐一个稳健实现策略

1️⃣ 计算 QEF 解
2️⃣ 如果 rank < 3 → fallback cell center
3️⃣ 如果 解不在 cell → clamp
4️⃣ 如果 clamp 后误差过大 → fallback

🎯 终极总结

你这个 artifact 典型原因是：

QEF 数值不稳定 + 没做 cell 约束

修好这一点：

壶身会立刻平滑

壶柄会正常

盖子不会爆裂