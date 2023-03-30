clear all;
clc;
Time=10;%%����ʱ�䣬s
dt=0.01;%%���沽����s
%% ��������
P=5884;   %%������N
rou=1.225;   %%�ܶȣ�kg/m3
S=4.72;   %%�ο����,m2
L=2.33;  %%�ο�����,m
Jz=4200; %%ת��������kg*m2
Isp=2097;   %%�ȳ壬N*s/kg
g=9.8;   %%�������ٶȣ�m/s2
a=340;   %%���٣�m/s
%% ״̬����
V=zeros(1,Time/dt);%%m/s���ٶ�
theta=zeros(1,Time/dt);   %%rad���������
wz=zeros(1,Time/dt);%rad/s���������ٶ�
op_theta=zeros(1,Time/dt);   %%rad��������
x=zeros(1,Time/dt);   %%m��ˮƽλ��
y=zeros(1,Time/dt);  %%m���߶�
m=zeros(1,Time/dt);   %%kg
alpha=zeros(1,Time/dt);   %%rad������
%%  ��ֵ����
Array_alpha=[-2 0 2 4 8]*pi/180;
Array_Cx=[0.02181 0.01961 0.02181 0.02903 0.06310];
Array_Cyalpha=[-0.0666 0 0.0670 0.1479 0.3540];
Array_mzalpha=[0.00524 0.00027 -0.00622 -0.02044 -0.07159];
KCx=polyfit(Array_alpha,Array_Cx,3);
KCyalpha=polyfit(Array_alpha,Array_Cyalpha,3);
Cydeltaz=0.00455;%ÿ�������������ϵ���仯
Kmzalpha=polyfit(Array_alpha,Array_mzalpha,3);
mzdeltaz=-0.0068;%ÿ��������ĸ�������ϵ���仯
%%
Array_deltaz=[-3,-1,0,1,3];
for i=1:5
    deltaz=Array_deltaz(i);
    %% ״̬������ʼֵ
    V(1)=0.9*a;
    theta(1)=0;   %%rad
    wz(1)=0;%rad/s
    op_theta(1)=3*pi/180;   %%rad
    x(1)=0;   %%m
    y(1)=20;  %%m
    m(1)=2000;   %%kg
    alpha(1)=3*pi/180;   %%rad
    for k=1:Time/dt
     %% ��̬����
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
    %% ��ͼ
    z=(0:Time/dt)*0.001;
    if i==1
        figure(1)%����
        alpha=alpha*180/pi;
        plot(z,alpha);hold on;
        figure(2)%�켣
        plot(z,y);hold on
        figure(3)%������
        op_theta=op_theta*180/pi;
        plot(z,op_theta);hold on;
        figure(4)%�������ٶ�
        wz=wz*180/pi;
        plot(z,wz);hold on;
        figure(5)%�������
        theta=theta*180/pi;
        plot(z,theta);hold on;
    elseif i== 2
        figure(1)
        alpha=alpha*180/pi;
        plot(z,alpha);hold on;
        figure(2)
        plot(z,y);hold on
        figure(3)%������
        op_theta=op_theta*180/pi;
        plot(z,op_theta);hold on;
        figure(4)%�����ǽ��ٶ�
        wz=wz*180/pi;
        plot(z,wz);hold on;
        figure(5)%�������
        theta=theta*180/pi;
        plot(z,theta);hold on;
    elseif i==3
        figure(1)
        alpha=alpha*180/pi;
        plot(z,alpha);hold on;
        figure(2)
        plot(z,y);hold on
        figure(3)%������
        op_theta=op_theta*180/pi;
        plot(z,op_theta);hold on;
        figure(4)%�����ǽ��ٶ�
        wz=wz*180/pi;
        plot(z,wz);hold on;
        figure(5)%�������
        theta=theta*180/pi;
        plot(z,theta);hold on;
    elseif i==4
        figure(1)
        alpha=alpha*180/pi;
        plot(z,alpha);hold on;
        figure(2)
        plot(z,y);hold on
        figure(3)%������
        op_theta=op_theta*180/pi;
        plot(z,op_theta);hold on;
        figure(4)%�����ǽ��ٶ�
        wz=wz*180/pi;
        plot(z,wz);hold on;
        figure(5)%�������
        theta=theta*180/pi;
        plot(z,theta);hold on;
    elseif i==5
        figure(1)
        alpha=alpha*180/pi;
        plot(z,alpha);hold on;
        legend('deltaz=-3��','deltaz=-1��','deltaz=0��','deltaz=1��','deltaz=3��')
%         title('����');
        xlabel('ʱ��/s')
        ylabel('����/deg')
        figure(2)
        plot(z,y);hold on
        legend('deltaz=-3��','deltaz=-1��','deltaz=0��','deltaz=1��','deltaz=3��')
        xlabel('ʱ��/s')
        ylabel('�߶�/m')
        figure(3)%������
        op_theta=op_theta*180/pi;
        plot(z,op_theta);hold on;
        legend('deltaz=-3��','deltaz=-1��','deltaz=0��','deltaz=1��','deltaz=3��')
        xlabel('ʱ��/s')
        ylabel('������/deg')
        figure(4)%�����ǽ��ٶ�
        wz=wz*180/pi;
        plot(z,wz);hold on;
        legend('deltaz=-3��','deltaz=-1��','deltaz=0��','deltaz=1��','deltaz=3��')
        xlabel('ʱ��/s')
        ylabel('�����ǽ��ٶ�/deg/s')
        figure(5)%�������
        theta=theta*180/pi;
        plot(z,theta);hold on;
        legend('deltaz=-3��','deltaz=-1��','deltaz=0��','deltaz=1��','deltaz=3��')
        xlabel('ʱ��/s')
        ylabel('�������/deg')
    end
end
