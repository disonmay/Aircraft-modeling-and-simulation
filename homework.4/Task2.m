clear all;
clc;
Time=120;
dt=0.01;
step=Time/dt;
%% ��������
P=5884;   %%������N
rou=1.225;   %%�ܶȣ�kg/m3
S=4.72;   %%�ο����,m2
L=2.33;  %%�ο�����,m
Jz=4200; %%ת��������kg*m2
Isp=2097;   %%�ȳ壬N*s/kg
g=9.8;   %%�������ٶȣ�m/s2
a=340;   %%���٣�m/s
Cydeltaz=0.00455/(pi/180);%ÿ�������������ϵ���仯
mzdeltaz=-0.0068/(pi/180);%ÿ��������ĸ�������ϵ���仯
%% ״̬����
V=zeros(1,step);%%m/s���ٶ�
theta=zeros(1,step);   %%rad���������
wz=zeros(1,step);%rad/s���������ٶ�
op_theta=zeros(1,step);   %%rad��������
x=zeros(1,step);   %%m��ˮƽλ��
y=zeros(1,step);  %%m���߶�
m=zeros(1,step);   %%kg
alpha=zeros(1,step);   %%rad������
deltaz=zeros(1,step);  %rad,��ƫ��
nr=zeros(1,step);  %���� m*a
I=zeros(1,step);  %�������
%% ״̬������ʼֵ
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
%% ����������
yr1=200;  %m,�����׶ε��ȶ��߶�
yr2=10;  %m,�½��׶ε��ȶ��߶�
K1=0.02;  %�׶�a�Ŀ�������
Kp1=0.001;Ki1=0.0000001;Kd1=0.001;  %�׶�b��c�ıջ�����
K2=0.02;  %�׶�d�Ŀ�������
Kp2=0.002;Ki2=0.000001;Kd2=0.01;  %�׶�e�ıջ�����
threshold=50;  %���ô������С����л�ʱ�ĸ߶Ȳ�
ws=0;b=0;
%%
for k=1:step-1
    %% ������
    if y(k)>=10 && y(k)<=yr1-threshold && k<30/dt    %�׶�A:����10~150m
        op_theta(k)=K1*k*dt+3*pi/180;
    end
    if y(k)>=yr1-threshold  && k<=ws+60/dt %�׶�B��С���+�߿�Ѳ����150m
        if y(k-1)<195 && y(k)>=195  %%���ɻ�������195m���ң����ɼ���ɻ���ʼ�߿�Ѳ�����������60s
            ws=k;
        end
        I(k)=I(k-1)+(yr1-y(k));
        op_theta(k)=Kp1*(yr1-y(k))+Ki1*I(k)+Kd1*((yr1-y(k))-(yr1-y(k-1)))/dt+2.6*pi/180;
        %%%ƽ�⹥��ѡ��̶�ֵ2.6deg������ڵ�ʵ������ͨ��PID���ڲ���
        Nop_theta=op_theta(k);
        b=k;
    end
    if y(k)>=yr2+threshold && k>=ws+60/dt %�׶�D:����200~80m
        op_theta(k)=-K2*(k-b)*dt+Nop_theta;
    end
    if y(k)<=yr2+threshold && k>=ws+60/dt  %�׶�EF��С���+�Ϳ�Ѳ��
        I(k)=I(k-1)+(yr2-y(k));
        op_theta(k)=Kp2*(yr2-y(k))+Ki2*I(k)+Kd2*(y(k-1)-y(k))/dt+2.6*(pi/180);
    end
    %% ��̬����
     %%%  ����
    alpha(k)=op_theta(k)-theta(k);
    if alpha(k)<-2*pi/180
        alpha(k)=-2*pi/180;
    elseif alpha(k)>8*pi/180
        alpha(k)=8*pi/180;
    end
    %%%  ��ƫ��
    deltaz(k)=-Valuemz_alpha(alpha(k))*alpha(k)/mzdeltaz;  %Ӧ��������,���Ǻ�ȷ��
    %%%  �ٶ�
    q(k)=rou*V(k)^2/2;
    Cx(k)=ValueCx(alpha(k));
    X(k)=Cx(k)*q(k)*S;
    dV(k)=(P*cos(alpha(k))-X(k))/m(k)-g*sin(theta(k));
    nr(k)=(P*cos(alpha(k))-X(k))/(m(k)*g);
    V(k+1)=V(k)+dV(k)*dt;
    %%%  �������
    Cy(k)=ValueCy_alpha(alpha(k))+Cydeltaz*deltaz(k);
    Y(k)=Cy(k)*q(k)*S;
    G(k)=m(k)*g;
    dtheta(k)=(P*sin(alpha(k))+Y(k)-G(k)*cos(theta(k)))/(m(k)*V(k));
    theta(k+1)=theta(k)+dtheta(k)*dt;
    %%%  ˮƽλ��
    dx(k)=V(k)*cos(theta(k));
    x(k+1)=x(k)+dx(k)*dt;
    %%%  �߶�
    dy(k)=V(k)*sin(theta(k));
    y(k+1)=y(k)+dy(k)*dt;
    %%%  ����
    mc=P/Isp;
    dm(k)=-mc;
    m(k+1)=m(k)+dm(k)*dt;
end
%% ��ͼ
z=(0:step-1)*dt;
figure(1)
plot(z,y)
xlabel('ʱ��/s')
ylabel('�߶�/m')
figure(2)
plot(z,op_theta*180/pi)
xlabel('ʱ��/s')
ylabel('������/deg')
figure(3)
plot(z,alpha*180/pi)
xlabel('ʱ��/s')
ylabel('����/deg')
figure(4)
plot(z,deltaz*180/pi)
xlabel('ʱ��/s')
ylabel('��ƽ��ƫ��/m')
figure(5)
plot(z,nr)
xlabel('ʱ��/s')
ylabel('����/G')