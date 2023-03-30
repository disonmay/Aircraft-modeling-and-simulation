clear all;
clc;
Time=10;%%仿真时间，s
dt=0.01;%%仿真步长，s
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
Array_deltaz=[-3,-1,0,1,3];
for i=1:5
    deltaz=Array_deltaz(i);
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
    end
    %% 作图
    z=(0:Time/dt)*0.001;
    if i==1
        figure(1)%攻角
        alpha=alpha*180/pi;
        plot(z,alpha);hold on;
        figure(2)%轨迹
        plot(z,y);hold on
        figure(3)%俯仰角
        op_theta=op_theta*180/pi;
        plot(z,op_theta);hold on;
        figure(4)%俯仰角速度
        wz=wz*180/pi;
        plot(z,wz);hold on;
        figure(5)%弹道倾角
        theta=theta*180/pi;
        plot(z,theta);hold on;
    elseif i== 2
        figure(1)
        alpha=alpha*180/pi;
        plot(z,alpha);hold on;
        figure(2)
        plot(z,y);hold on
        figure(3)%俯仰角
        op_theta=op_theta*180/pi;
        plot(z,op_theta);hold on;
        figure(4)%俯仰角角速度
        wz=wz*180/pi;
        plot(z,wz);hold on;
        figure(5)%弹道倾角
        theta=theta*180/pi;
        plot(z,theta);hold on;
    elseif i==3
        figure(1)
        alpha=alpha*180/pi;
        plot(z,alpha);hold on;
        figure(2)
        plot(z,y);hold on
        figure(3)%俯仰角
        op_theta=op_theta*180/pi;
        plot(z,op_theta);hold on;
        figure(4)%俯仰角角速度
        wz=wz*180/pi;
        plot(z,wz);hold on;
        figure(5)%弹道倾角
        theta=theta*180/pi;
        plot(z,theta);hold on;
    elseif i==4
        figure(1)
        alpha=alpha*180/pi;
        plot(z,alpha);hold on;
        figure(2)
        plot(z,y);hold on
        figure(3)%俯仰角
        op_theta=op_theta*180/pi;
        plot(z,op_theta);hold on;
        figure(4)%俯仰角角速度
        wz=wz*180/pi;
        plot(z,wz);hold on;
        figure(5)%弹道倾角
        theta=theta*180/pi;
        plot(z,theta);hold on;
    elseif i==5
        figure(1)
        alpha=alpha*180/pi;
        plot(z,alpha);hold on;
        legend('deltaz=-3°','deltaz=-1°','deltaz=0°','deltaz=1°','deltaz=3°')
%         title('攻角');
        xlabel('时间/s')
        ylabel('攻角/deg')
        figure(2)
        plot(z,y);hold on
        legend('deltaz=-3°','deltaz=-1°','deltaz=0°','deltaz=1°','deltaz=3°')
        xlabel('时间/s')
        ylabel('高度/m')
        figure(3)%俯仰角
        op_theta=op_theta*180/pi;
        plot(z,op_theta);hold on;
        legend('deltaz=-3°','deltaz=-1°','deltaz=0°','deltaz=1°','deltaz=3°')
        xlabel('时间/s')
        ylabel('俯仰角/deg')
        figure(4)%俯仰角角速度
        wz=wz*180/pi;
        plot(z,wz);hold on;
        legend('deltaz=-3°','deltaz=-1°','deltaz=0°','deltaz=1°','deltaz=3°')
        xlabel('时间/s')
        ylabel('俯仰角角速度/deg/s')
        figure(5)%弹道倾角
        theta=theta*180/pi;
        plot(z,theta);hold on;
        legend('deltaz=-3°','deltaz=-1°','deltaz=0°','deltaz=1°','deltaz=3°')
        xlabel('时间/s')
        ylabel('弹道倾角/deg')
    end
end
