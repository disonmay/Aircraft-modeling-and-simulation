clear all;
clc;
%%
P=5884;   %%推力，N
rou=1.225;   %%密度，kg/m3
Sr=4.72;   %%参考面积,m2
Lr=2.33;  %%参考长度,m
Jz=4200; %%转动惯量，kg*m2
Isp=2097;   %%比冲，N*s/kg
g=9.8;   %%重力加速度，m/s2
a=340;   %%声速，m/s
m=2000;
V=0.9*a;
q=0.5*rou*V^2;
mz_deltaz=-0.0068;
%%
mz_wz=0;
mz_alpha=(-0.00622-0.02044)/2;
Cy_alpha=(0.0670+0.1479)/2;
%%
a_deltaz=mz_deltaz*q*Sr*Lr/Jz;
b_alpha=(P+Cy_alpha*q*Sr)/(m*V);
a_wz=mz_wz*q*Sr*Lr/Jz;
a_alpha=mz_alpha*q*Sr*Lr/Jz;
num1=a_deltaz;
num2=a_deltaz*b_alpha;
den1=b_alpha-a_wz;
den2=-a_alpha-a_wz*b_alpha;
%%
Kp=300;
Kd=50;
%传递函数
Gr=tf([0.5*0.2743 0.5*124.7],[1.32*10^(-5) 0.0082 1.549 62.33]);%舵机的传递函数
Gb=tf([-num1 -num2],[1 den1 den2]);%舵偏角到俯仰角速率的传递函数
% s1=tf([1],[1 0]);
% Gs=Kp*Gr*Gb*s1;
% H=tf([Kd Kp],[Kp]);%对PD姿态控制回路做了等效设计反馈设计
% opesys=Gs*H;
G=Gr*Gb;
C=tf([Kp],[1,0]);
H=tf([Kd,Kp],[Kp]);
opesys=C*G*H;
figure(1)
margin(opesys);
% figure(2)
% margin(Gr);
% figure(3)
% margin(Gb);
