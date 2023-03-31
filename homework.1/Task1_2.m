clear all;
clc;
Time=10;%%仿真时间，s
dt=0.001;%%仿真步长，s
%% 常量参数
P=5884;   %%推力，N
rou=1.225;   %%密度，kg/m3
S=4.72;   %%参考面积,m2
L=2.33;  %%参考长度,m
Jz=4200; %%转动惯量，kg*m2
Isp=2097;   %%比冲，N*s/kg
g=9.8;   %%重力加速度，m/s2
a=340;   %%声速，m/s
%% 状态变量
V=zeros(1,Time/dt);%%m/s，速度
theta=zeros(1,Time/dt);   %%rad，弹道倾角
wz=zeros(1,Time/dt);%rad/s，俯仰角速度
op_theta=zeros(1,Time/dt);   %%rad，俯仰角
x=zeros(1,Time/dt);   %%m，水平位置
y=zeros(1,Time/dt);  %%m，高度
m=zeros(1,Time/dt);   %%kg
alpha=zeros(1,Time/dt);   %%rad，攻角
%%  插值运算
Array_alpha=[-2 0 2 4 8]*pi/180;
Array_Cx=[0.02181 0.01961 0.02181 0.02903 0.06310];
Array_Cyalpha=[-0.0666 0 0.0670 0.1479 0.3540];
Array_mzalpha=[0.00524 0.00027 -0.00622 -0.02044 -0.07159];
KCx=polyfit(Array_alpha,Array_Cx,3);
KCyalpha=polyfit(Array_alpha,Array_Cyalpha,3);
Cydeltaz=0.00455;%每弧度引起的升力系数变化
Kmzalpha=polyfit(Array_alpha,Array_mzalpha,3);
mzdeltaz=-0.0068;%每弧度引起的俯仰力矩系数变化
%%
deltaz=1;%%deg，舵偏角
%% 状态变量初始值
V(1)=0.9*a;
theta(1)=0;   %%rad
wz(1)=0;%rad/s
op_theta(1)=3*pi/180;   %%rad
x(1)=0;   %%m
y(1)=20;  %%m
m(1)=2000;   %%kg
alpha(1)=3*pi/180;   %%rad
for k=1:Time/dt
    %% 动态方程
    %%%
    q(k)=rou*V(k)^2/2;
    Cx(k)=polyval(KCx,alpha(k));
    X(k)=Cx(k)*q(k)*S;
    dV(k)=(P*cos(alpha(k))-X(k))/m(k)-g*sin(theta(k));
    V(k+1)=V(k)+dV(k)*dt;
    %%%
    Cy(k)=polyval(KCyalpha,alpha(k))+Cydeltaz*deltaz;
    Y(k)=Cy(k)*q(k)*S;
    G(k)=m(k)*g;
    dtheta(k)=(P*sin(alpha(k))+Y(k)-G(k)*cos(theta(k)))/(m(k)*V(k));
    theta(k+1)=theta(k)+dtheta(k)*dt;
    %%%
    mz(k)=polyval(Kmzalpha,alpha(k))+mzdeltaz*deltaz;
    Mz(k)=mz(k)*q(k)*S*L;
    dwz(k)=Mz(k)/Jz;
    wz(k+1)=wz(k)+dwz(k)*dt;
    %%%
    dphi(k)=wz(k);
    op_theta(k+1)=op_theta(k)+dphi(k)*dt;
    %%%
    dx(k)=V(k)*cos(theta(k));
    x(k+1)=x(k)+dx(k)*dt;
    %%%
    dy(k)=V(k)*sin(theta(k));
    y(k+1)=y(k)+dy(k)*dt;
    %%%
    mc=P/Isp;
    dm(k)=-mc;
    m(k+1)=m(k)+dm(k)*dt;
    %%%
    alpha(k+1)=op_theta(k+1)-theta(k+1);
    alphadot(k)=dphi(k)-dtheta(k);
end
%% 攻角辨识
for i=1:Time/dt-1
    alphadot2(i)=(alphadot(i+1)-alphadot(i))/dt;
end
YN=alphadot2';
for i=1:Time/dt-1
    phiN(i,:)=[alpha(i),alphadot(i),deltaz];
end
bs_alpha=inv(phiN'*phiN)*phiN'*YN;
x1=[];%alpha
x2=[];%alphadot
x1(1)=alpha(1);
x2(1)=alphadot(1);
for i=1:Time/dt
    x1dot=x2(i);
    x2dot=bs_alpha(1)*x1(i)+bs_alpha(2)*x2(i)+bs_alpha(3)*deltaz;
    x1(i+1)=x1(i)+x1dot*dt;
    x2(i+1)=x2(i)+x2dot*dt; 
end
%作图
figure(1)
z=(0:Time/dt)*0.001;
plot(z,alpha*180/pi,z,x1*180/pi)
legend('模型攻角','辨识攻角')
xlabel('时间/s')
ylabel('攻角/deg')
%% 俯仰角速度
%计算角速度二阶导
for i=1:Time/dt-1
    wzdot2(i)=(dwz(i+1)-dwz(i))/dt;
end
YN=wzdot2';
for i=1:Time/dt-1
    phiN(i,:)=[wz(i),dwz(i),deltaz];
end
 bs_wz=inv(phiN'*phiN)*phiN'*YN;%辨识结果
%用辨识结果计算
x1=[];%wz
x2=[];%dwz
x1(1)=wz(1);
x2(1)=dwz(1);
for i=1:Time/dt
    x1dot=x2(i);
    x2dot=bs_wz(1)*x1(i)+bs_wz(2)*x2(i)+bs_wz(3)*deltaz;
    x1(i+1)=x1(i)+x1dot*dt;
    x2(i+1)=x2(i)+x2dot*dt; 
end
%作图
figure(2)
z=(0:Time/dt)*0.001;
plot(z,wz*180/pi,z,x1*180/pi)
legend('模型俯仰角速度','辨识俯仰角速度')
xlabel('时间/s')
ylabel('角速度/deg/s')
%% 俯仰角
Pitch(1)=op_theta(1);
for i=1:Time/dt
    Pitch(i+1)=Pitch(i)+x1(i)*dt;
end
figure(3)
z=(0:Time/dt)*0.001;
plot(z,op_theta*180/pi,z,Pitch*180/pi)
legend('模型俯仰角','辨识俯仰角')
xlabel('时间/s')
ylabel('俯仰角/deg')
%%  计算a,b,c
a_alpha=1/bs_alpha(3);
b_alpha=-a_alpha*bs_alpha(2);
c_alpha=-a_alpha*bs_alpha(1);
fprintf('攻角辨识结果： a=%.4f,b=%.4f,c=%.4f;\n',a_alpha,b_alpha,c_alpha)
a_wz=1/bs_wz(3);
b_wz=-a_wz*bs_wz(2);
c_wz=-a_wz*bs_wz(1);
fprintf('角速度辨识结果： a=%.4f,b=%.4f,c=%.4f;\n',a_wz,b_wz,c_wz)