%��������ҵ������̬�����ȶ�ԣ�Ȳ�����
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
mzaf=sd(1);%����Mz�Ԧ��󵼵�ԭһ������ϵ�����õ���幥�ǲ�֪�����о�ֱ��������mzalpha�е㲻���ʣ�Ӧ�ö��㿼�ǣ�ĳ�������µ������
Cy_alpha=[-0.066 0 0.067 0.1479 0.354];
sdd=polyfit(alpha,Cy_alpha,1);
Cyaf=sdd(1);%����Y�Ԧ��󵼵�ԭһ������ϵ��

mzdz=-0.0068;%����Ч��
mzwz=0;
m=2000;
V=0.9*a;
P=5884;
q=0.5*1.225*V*V;%��ѹͷ
Sr=4.72;
Lr=2.33;
Jz=4200;
%���Ի�ʱ���ݺ����ļ�������
aaf=mzaf*q*Sr*Lr/Jz;
adz=mzdz*q*Sr*Lr/Jz;
awz=mzwz*q*Sr*Lr/Jz;%����w���ڸ������ص�Ӱ��
baf=(P+Cyaf*q*Sr)/(m*V);

%���ݺ���
Gr=0.5*(0.2743*s+124.7)/((1.32*10^(-5))*s^3+0.0082*s^2+1.549*s+62.33);%����Ĵ��ݺ���
Gb=-(adz*s+adz*baf)/(s^2+(baf-awz)*s+(-awz*baf-aaf));%��ƫ�ǵ����������ʵĴ��ݺ���
s1=1/s;
Gs=Kp*Gr*Gb*s1;
H=(Kd*s+Kp)/Kp;%��PD��̬���ƻ�·���˵�Ч��Ʒ������

G1=1+Gs*H*(A*cos(phase)-1j*A*sin(phase));%�����ȶ�ԣ�Ȳ����ӵĿ������ݺ���
%�ٽ��ȶ�A=1��phase=0
G2=subs(G1,[s A phase],[1j*w 1 0]);%subs,���ż��㺯������R = subs(S, old, new) ����new��ֵ������ű��ʽ��old��ֵ��
                                   %oldΪ���ű��������ַ�����������new��һ�����Ż���ֵ��������ʽ��
real1=real(G2);%ȡʵ��
imag1=imag(G2);%ȡ�鲿
real2=subs(real1,w,wt);%ת���ɺܶ����ʽ������w��������ֵ
imag2=subs(imag1,w,wt);
%��λԣ��A=1��phase=pi/4
G3=subs(G1,[s A phase],[1j*w 1 pi/4]);
real3=real(G3);
imag3=imag(G3);
real4=subs(real3,w,wt);
imag4=subs(imag3,w,wt);
%��ֵԣ��A=2��phase=0
G4=subs(G1,[s A phase],[1j*w 2 0]);
real5=real(G4);
imag5=imag(G4);
real6=subs(real5,w,wt);
imag6=subs(imag5,w,wt);
for k=1:length(wt)
    %%%%%�ٽ��ȶ�
    y1s=real2(k);
    y2s=imag2(k);
    sols=vpasolve(y1s,y2s);%�ⷽ�̣���δ֪���ŵķ���
    kpss1(k)=double(sols.Kp);
    kdss1(k)=double(sols.Kd);
    %%%%%��λԣ��
    y1s=real4(k);
    y2s=imag4(k);
    sols=vpasolve(y1s,y2s);
    kpss2(k)=double(sols.Kp);
    kdss2(k)=double(sols.Kd);
    %%%%%��ֵԣ��
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
legend('�ٽ��ȶ�','��λԣ��45��','��ֵԣ��6dB');
xlabel('kp');
ylabel('kd');
axis([0,3000,0,300]);
hold on
