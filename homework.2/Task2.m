clear all;
clc;
Time=120;
dt=0.01;
step=Time/dt;
%% 常量参数
P=5884;   %%推力，N
rou=1.225;   %%密度，kg/m3
S=4.72;   %%参考面积,m2
L=2.33;  %%参考长度,m
Jz=4200; %%转动惯量，kg*m2
Isp=2097;   %%比冲，N*s/kg
g=9.8;   %%重力加速度，m/s2
a=340;   %%声速，m/s
Cydeltaz=0.00455/(pi/180);%每弧度引起的升力系数变化
mzdeltaz=-0.0068/(pi/180);%每弧度引起的俯仰力矩系数变化
%% 状态变量
V=zeros(1,step);%%m/s，速度
theta=zeros(1,step);   %%rad，弹道倾角
wz=zeros(1,step);%rad/s，俯仰角速度
op_theta=zeros(1,step);   %%rad，俯仰角
x=zeros(1,step);   %%m，水平位置
y=zeros(1,step);  %%m，高度
m=zeros(1,step);   %%kg
alpha=zeros(1,step);   %%rad，攻角
deltaz=zeros(1,step);  %rad,舵偏角
nr=zeros(1,step);  %过载 m*a
I=zeros(1,step);  %积分误差
%% 状态变量初始值
V(1)=0.9*a;
theta(1)=0;   %%rad
wz(1)=0;%rad/s
op_theta(1)=3*pi/180;   %%rad
x(1)=0;   %%m
y(1)=10;  %%m
m(1)=2000;   %%kg
alpha(1)=3*pi/180;   %%rad
deltaz(1)=-1.96*pi/180;
nr(1)=0;
alpha_b=zeros(1,step);
%% 控制器参数
yr1=200;  %m,爬升阶段的稳定高度
yr2=10;  %m,下降阶段的稳定高度
K1=0.02;  %阶段a的开环参数
Kp1=0.001;Ki1=0.0000001;Kd1=0.001;  %阶段b、c的闭环参数
K2=0.02;  %阶段d的开环参数
Kp2=0.002;Ki2=0.000001;Kd2=0.01;  %阶段e的闭环参数
threshold=50;  %设置大误差与小误差切换时的高度差
ws=0;b=0;
%%
for k=1:step-1
    %% 控制律
    if y(k)>=10 && y(k)<=yr1-threshold && k<30/dt    %阶段A:大误差，10~150m
        op_theta(k)=K1*k*dt+3*pi/180;
    end
    if y(k)>=yr1-threshold  && k<=ws+60/dt %阶段B：小误差+高空巡航，150m
        if y(k-1)<195 && y(k)>=195  %%当飞机上升至195m左右，即可假设飞机开始高空巡航，让其飞行60s
            ws=k;
        end
        I(k)=I(k-1)+(yr1-y(k));
        op_theta(k)=Kp1*(yr1-y(k))+Ki1*I(k)+Kd1*((yr1-y(k))-(yr1-y(k-1)))/dt+2.6*pi/180;
        %%%平衡攻角选择固定值2.6deg，其存在的实际误差可通过PID环节补偿
        Nop_theta=op_theta(k);
        b=k;
    end
    if y(k)>=yr2+threshold && k>=ws+60/dt %阶段D:大误差，200~80m
        op_theta(k)=-K2*(k-b)*dt+Nop_theta;
    end
    if y(k)<=yr2+threshold && k>=ws+60/dt  %阶段EF：小误差+低空巡航
        I(k)=I(k-1)+(yr2-y(k));
        op_theta(k)=Kp2*(yr2-y(k))+Ki2*I(k)+Kd2*(y(k-1)-y(k))/dt+2.6*(pi/180);
    end
    %% 动态方程
     %%%  攻角
    alpha(k)=op_theta(k)-theta(k);
    if alpha(k)<-2*pi/180
        alpha(k)=-2*pi/180;
    elseif alpha(k)>8*pi/180
        alpha(k)=8*pi/180;
    end
    %%%  舵偏角
    deltaz(k)=-Valuemz_alpha(alpha(k))*alpha(k)/mzdeltaz;  %应该是这样,不是很确定
    %%%  速度
    q(k)=rou*V(k)^2/2;
    Cx(k)=ValueCx(alpha(k));
    X(k)=Cx(k)*q(k)*S;
    dV(k)=(P*cos(alpha(k))-X(k))/m(k)-g*sin(theta(k));
    nr(k)=(P*cos(alpha(k))-X(k))/(m(k)*g);
    V(k+1)=V(k)+dV(k)*dt;
    %%%  弹道倾角
    Cy(k)=ValueCy_alpha(alpha(k))+Cydeltaz*deltaz(k);
    Y(k)=Cy(k)*q(k)*S;
    G(k)=m(k)*g;
    dtheta(k)=(P*sin(alpha(k))+Y(k)-G(k)*cos(theta(k)))/(m(k)*V(k));
    theta(k+1)=theta(k)+dtheta(k)*dt;
    %%%  水平位置
    dx(k)=V(k)*cos(theta(k));
    x(k+1)=x(k)+dx(k)*dt;
    %%%  高度
    dy(k)=V(k)*sin(theta(k));
    y(k+1)=y(k)+dy(k)*dt;
    %%%  质量
    mc=P/Isp;
    dm(k)=-mc;
    m(k+1)=m(k)+dm(k)*dt;
end
%% 绘图
z=(0:step-1)*dt;
figure(1)
plot(z,y)
xlabel('时间/s')
ylabel('高度/m')
figure(2)
plot(z,op_theta*180/pi)
xlabel('时间/s')
ylabel('俯仰角/deg')
figure(3)
plot(z,alpha*180/pi)
xlabel('时间/s')
ylabel('攻角/deg')
figure(4)
plot(z,deltaz*180/pi)
xlabel('时间/s')
ylabel('配平舵偏角/m')
figure(5)
plot(z,nr)
xlabel('时间/s')
ylabel('过载/G')