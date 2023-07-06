首先，我们来看看根据DH方程求关节角度的步骤：

确定坐标系构建的k-1->k连接向量$P_{k-1k}$

计算$T^z_{k-1}(q_k)\cdot T^x_{k-1}(d_k)$

通过求解雅可比矩阵 $J$ 关于 $q_k$ 的齐次方程 $\hat{R}{k~k+1}\cdot[J]=P{k-1k}$ 来求解关节角度

 

完整的DH方程式如下：

$T_{i+1i} =
\begin{bmatrix}
-c_{i+1} & -s_{i+1} c_a & s_{i+1}s_a & a_ic_{i+1} \
s_{i+1} & -c_{i+1}c_a & -c_{i+1}s_a & a_is_{i+1} \
0 & s_a & c_a & d_i \
0 & 0 & 0 & 1
\end{bmatrix}
$

其中，$c_{i+1} = cos(q_{i+1})$, $s_{i+1} = sin(q_{i+1})$, $c_a = cos(\alpha_i)$, $s_a = sin(\alpha_i)$

根据给定的DH参数，我们能够求出这个6轴机械臂每个关节角度的表达式：

$q_1=-atan2(\frac{-P_{02}}{P_{00}},\frac{P_{01}}{P_{00}})$

$q_2=acos(\frac{P_{20}P_{00}-P_{10}P_{01}}{\sqrt{(P_{20}P_{00}-P_{10}P_{01})^2+(P_{21}P_{00}-P_{11}P_{01})^2}})$

$q_3=asin(\frac{-P_{03}}{\sqrt{P_{02}^2+P_{03}^2}})$

$q_4=atan2(-P_{13},P_{33})$

$q_5=acos(\frac{P_{30}P_{10}-P_{20}P_{20}}{\sqrt{(P_{30}P_{10}-P_{20}P_{20})^2+(P_{31}P_{10}-P_{21}P_{20})^2}})$

$q_6=atan2(-P_{21},P_{11})$

为了计算上述6轴机械臂的逆运动学表达式，可以使用Denavit-Hartenberg (DH) 参数表：
$$T_0^6 = T_0^1(q_1)[d_1]T_1^2 (q_2)[a_2] T_2^3 (q_3)[a_3] T_3^4 (q_4)[d_4]T_4^5 (q_5)[a_5]T_5^6 (q_6)[d_6]$$

其中每一个关节相对应的矩阵为：
$$
T(q_i)= \begin{pmatrix}
\cos {q_i} & -\sin {q_i} & 0 & a_i \
\sin {q_i} & \cos {q_i} & 0 & b_i\
0 & 0 & 1 & d_i \
0 & 0 & 0 & 1
\end{pmatrix}
$$

因此，该机械臂的DH参数表为：

i	$q_{i}$	$d_{i}$	$a_{i}$	$\alpha_{i}$	$\theta_{i}$
1	$q_1$	100	0	-1.571	revolute
2	$q_2$	0	300	0	revolute
3	$q_3$	0	280	0	revolute
4	$q_4$	0	0	-1.571	revolute
5	$q_5$	200	0	1.571	revolute
6	$q_6$	0	100	1.571	revolute
因此，上述机械臂的逆运动学的表达式可以写成：
void inverseKinematics(double q[6], double t[4][4]) {
    // DH parameters associated with each joint
    double d1=100; 
    double a2=300; 
    double a3=280; 
    double d4=0; 
    double a5=200;
    double a6=100;

    // Compute the joint angles
     q[0] = atan2(t[0][2], t[0][0]);
     q[1] = acos((-t[0][1] + d1)/a2);
     q[2] = acos(((-pow(t[0][1], 2) - pow(t[2][3], 2) + d1*d1 + a2*a2 + a3*a3) / 
              (2*a2*a3));
     q[3] = atan2(t[2][3], -t[0][1]) - q[2];
     q[4] = atan2(t[1][2], t[1][0]) - q[0];
     q[5] = atan2(t[2][1], t[2][0]) - q[0] - q[1]- q[2] - q[3];