%第三次作业俯仰姿态控制稳定裕度测试子
%2020/5/23
clc
close all
clear all
a=340;
syms   s w Kd Kp A phase;
wt=0.1:0.3:400;
l=length(wt);
kpss1=zeros(1,l);
kdss1=zeros(1,l);
kpss2=zeros(1,l);
kdss2=zeros(1,l);
kpss3=zeros(1,l);
kdss3=zeros(1,l);
mz_alpha=[0.00524 0.00027 -0.00622 -0.02044 -0.07159];
alpha=[-2 0 2 4 8]*pi/180;
sd=polyfit(alpha,mz_alpha,1);
mzaf=sd(1);%力矩Mz对α求导的原一次幂项系数，该点具体攻角不知道。感觉直接这样求mzalpha有点不合适，应该定点考虑，某个攻角下的情况。
Cy_alpha=[-0.066 0 0.067 0.1479 0.354];
sdd=polyfit(alpha,Cy_alpha,1);
Cyaf=sdd(1);%升力Y对α求导的原一次幂项系数

mzdz=-0.0068;%舵面效率
mzwz=0;
m=2000;
V=0.9*a;
P=5884;
q=0.5*1.225*V*V;%动压头
Sr=4.72;
Lr=2.33;
Jz=4200;
%线性化时传递函数的几个参数
aaf=mzaf*q*Sr*Lr/Jz;
adz=mzdz*q*Sr*Lr/Jz;
awz=mzwz*q*Sr*Lr/Jz;%忽略w对于俯仰力矩的影响
baf=(P+Cyaf*q*Sr)/(m*V);

%传递函数
Gr=0.5*(0.2743*s+124.7)/((1.32*10^(-5))*s^3+0.0082*s^2+1.549*s+62.33);%舵机的传递函数
Gb=-(adz*s+adz*baf)/(s^2+(baf-awz)*s+(-awz*baf-aaf));%舵偏角到俯仰角速率的传递函数
s1=1/s;
Gs=Kp*Gr*Gb*s1;
H=(Kd*s+Kp)/Kp;%对PD姿态控制回路做了等效设计反馈设计

G1=1+Gs*H*(A*cos(phase)-1j*A*sin(phase));%带有稳定裕度测试子的开环传递函数
%临界稳定A=1，phase=0
G2=subs(G1,[s A phase],[1j*w 1 0]);%subs,符号计算函数。用R = subs(S, old, new) 利用new的值代替符号表达式中old的值。
                                   %old为符号变量或是字符串变量名。new是一个符号货数值变量或表达式。
real1=real(G2);%取实部
imag1=imag(G2);%取虚部
real2=subs(real1,w,wt);%转化成很多个等式，其中w都被赋上值
imag2=subs(imag1,w,wt);
%相位裕度A=1，phase=pi/4
G3=subs(G1,[s A phase],[1j*w 1 pi/4]);
real3=real(G3);
imag3=imag(G3);
real4=subs(real3,w,wt);
imag4=subs(imag3,w,wt);
%幅值裕度A=2，phase=0
G4=subs(G1,[s A phase],[1j*w 2 0]);
real5=real(G4);
imag5=imag(G4);
real6=subs(real5,w,wt);
imag6=subs(imag5,w,wt);
for k=1:length(wt)
    %%%%%临界稳定
    y1s=real2(k);
    y2s=imag2(k);
    sols=vpasolve(y1s,y2s);%解方程，解未知符号的方程
    kpss1(k)=double(sols.Kp);
    kdss1(k)=double(sols.Kd);
    %%%%%相位裕度
    y1s=real4(k);
    y2s=imag4(k);
    sols=vpasolve(y1s,y2s);
    kpss2(k)=double(sols.Kp);
    kdss2(k)=double(sols.Kd);
    %%%%%幅值裕度
    y1s=real6(k);
    y2s=imag6(k);
    sols=vpasolve(y1s,y2s);
    kpss3(k)=double(sols.Kp);
    kdss3(k)=double(sols.Kd);
end
figure(3)
plot(kpss1,kdss1,'black','LineWidth',2);
hold on
plot(kpss2,kdss2,'b','LineWidth',2);
hold on
plot(kpss3,kdss3,'r','LineWidth',2);
legend('临界稳定','相位裕度45度','幅值裕度6dB');
xlabel('kp');
ylabel('kd');
axis([0,3000,0,300]);
hold on
