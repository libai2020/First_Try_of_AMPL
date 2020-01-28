# OCDT相关参数及集合 #
param Nfe := 10;
param K_radau := 3;
set I := {1..Nfe};
set I1:= {1..Nfe-1};
set J := {1..K_radau};
set K := {0..K_radau};
param tau{j in K};
param dljtauk{j in K,k in K};


# 车辆自身参数 #
param L_wheelbase := 2.8;
param a_max := 0.3;
param v_max := 2.0;
param phy_max := 0.72;
param w_max := 0.54;


# 决策变量 #
var x{i in I, j in K};
var y{i in I, j in K};
var theta{i in I, j in K};
var v{i in I, j in K};
var phy{i in I, j in K};
var a{i in I, j in J};
var w{i in I, j in J};
var tf >= 0;
var hi = tf / Nfe;


# 最小化代价函数 #
minimize cost_function:
10 * tf + sum{i in I, j in J}(w[i,j]^2);


# 车辆运动学微分方程组 #
s.t. DIFF_dxdt {i in I, k in J}:
sum{j in K}(dljtauk[j,k]*x[i,j]) = hi * v[i,k] * cos(theta[i,k]);

s.t. DIFF_dydt {i in I, k in J}:
sum{j in K}(dljtauk[j,k]*y[i,j]) = hi * v[i,k] * sin(theta[i,k]);

s.t. DIFF_dtdt {i in I, k in J}:
sum{j in K}(dljtauk[j,k]*theta[i,j]) = hi * tan(phy[i,k]) * v[i,k] / L_wheelbase;

s.t. DIFF_dvdt {i in I, k in J}:
sum{j in K}(dljtauk[j,k]*v[i,j]) = hi * a[i,k];

s.t. DIFF_dpdt {i in I, k in J}:
sum{j in K}(dljtauk[j,k]*phy[i,j]) = hi * w[i,k];

s.t. EQ_diffx {i in I1}:
x[i+1,0] = sum{j in K}((prod{k in K:k<>j}((1-tau[k])/(tau[j]-tau[k])))*x[i,j]);

s.t. EQ_diffy {i in I1}:
y[i+1,0] = sum{j in K}((prod{k in K:k<>j}((1-tau[k])/(tau[j]-tau[k])))*y[i,j]);

s.t. EQ_difftheta {i in I1}:
theta[i+1,0] = sum{j in K}((prod{k in K:k<>j}((1-tau[k])/(tau[j]-tau[k])))*theta[i,j]);

s.t. EQ_diffv {i in I1}:
v[i+1,0] = sum{j in K}((prod{k in K:k<>j}((1-tau[k])/(tau[j]-tau[k])))*v[i,j]);

s.t. EQ_diffphy {i in I1}:
phy[i+1,0] = sum{j in K}((prod{k in K:k<>j}((1-tau[k])/(tau[j]-tau[k])))*phy[i,j]);


# 两点边值约束 #
s.t. EQ_init_x:
x[1,0] = 1.03;

s.t. EQ_init_y:
y[1,0] = 2.41;

s.t. EQ_init_theta:
theta[1,0] = -0.03;

s.t. EQ_init_v:
v[1,0] = 0.1;

s.t. EQ_init_phy:
phy[1,0] = -0.08;


s.t. EQ_terminal_x:
x[Nfe,K_radau] = 5.31;

s.t. EQ_terminal_y:
y[Nfe,K_radau] = 2.41;

s.t. EQ_terminal_theta:
theta[Nfe,K_radau] = -3.1416;

s.t. EQ_terminal_v:
v[Nfe,K_radau] = 0.0;

s.t. EQ_terminal_phy:
phy[Nfe,K_radau] = 0.0;

s.t. EQ_terminal_a:
a[Nfe,K_radau] = 0.0;

s.t. EQ_terminal_w:
w[Nfe,K_radau] = 0.0;


# 流形约束 #
s.t. Bonds_v {i in I,j in K}:
-v_max <= v[i,j] <= v_max;

s.t. Bonds_phy {i in I,j in K}:
-phy_max <= phy[i,j] <= phy_max;

s.t. Bonds_a {i in I,j in J}:
-a_max <= a[i,j] <= a_max;

s.t. Bonds_w {i in I,j in J}:
-w_max <= w[i,j] <= w_max;


data;
param: dljtauk :=
0	0	-9.0000
0	1	-4.1394
0	2	1.7394
0	3	-3
1	0	10.0488
1	1	3.2247
1	2	-3.5678
1	3	5.5320
2	0	-1.3821
2	1	1.1678
2	2	0.7753
2	3	-7.5320
3	0	0.3333
3	1	-0.2532
3	2	1.0532
3	3	5.0000;

param: tau :=
0	0
1	0.1550510257216822
2	0.6449489742783178
3	1.0;