原位置：

original_pose = [x, y, z, rx, ry, rz]

original_pose --> original_position_RT, 末端坐标系相对于基座坐标系的变换矩阵，因为末端点在末端坐标系原点

shelf_pose = [R_shelf2Cam, T_shelf2Cam, ...] 试管架坐标系

试管架坐标系 --> 基座坐标系，左乘

试管架位置（基座坐标系）= RT_EndToBase * RT_CamToEnd * RT_ShelfToCam * 试管架位置（试管架坐标系）

绕z轴旋转180度的右手坐标系下的旋转矩阵（Rotation Matrix，RT矩阵）可以表示为：

$$
 R_z(180^\circ) = \begin{pmatrix}
\cos(180^\circ) & -\sin(180^\circ) & 0 \\
\sin(180^\circ) & \cos(180^\circ) & 0 \\
0 & 0 & 1
\end{pmatrix} 
$$

由于 \(\cos(180^\circ) = -1\) 和 \(\sin(180^\circ) = 0\)，所以这个矩阵可以简化为：

$$ R_z(180^\circ) = \begin{pmatrix}
-1 & 0 & 0 \\
0 & -1 & 0 \\
0 & 0 & 1
\end{pmatrix} $$

这个矩阵表示绕z轴旋转180度的右手坐标系下的旋转矩阵。

绕 z 轴顺时针旋转 90 度的旋转矩阵（Rotation Matrix, RT 矩阵）可以表示为：
\cos(-90^\circ) & -\sin(-90^\circ) & 0 \\
\sin(-90^\circ) & \cos(-90^\circ) & 0 \\
0 & 0 & 1
\end{pmatrix}
由于 $$\cos(-90^\circ) = 0$$ 和 $$\sin(-90^\circ) = -1$$，所以这个矩阵可以简化为：
$$ R_z(-90^\circ) = \begin{pmatrix}
0 & 1 & 0 \\
-1 & 0 & 0 \\
0 & 0 & 1
\end{pmatrix} $$
这个旋转矩阵表示绕 z 轴顺时针旋转 90 度的右手坐标系下的旋转。
完整的旋转-平移（RT）矩阵可以表示为一个 4x4 的齐次变换矩阵，形式如下：
$$
RT = \begin{pmatrix}
0 & 1 & 0 & 0 \\
-1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{pmatrix}$$

绕x轴顺时针90度
$$
R_x\left(-\frac{\pi}{2}\right) = \begin{bmatrix}
1 & 0 & 0 \\
0 & 0 & 1 \\
0 & -1 & 0
\end{bmatrix}
$$