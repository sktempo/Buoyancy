clc
clear all
close all


Hs=2;
Tp=3;
dw=0.05;
w=0.2:dw:2;
s=487*(Hs^2)./((Tp^4).*(w.^5)).*exp(-1948./((Tp^4).*(w.^4)));
x=0:0.1:50;
dt=0.01;
t=0:dt:10;
K=(w.^2)./9.81;
phix=(2*pi)*rand(1,length(w));
amp=sqrt(2*dw*s);
sinx=zeros(length(x),length(w));
sinmult=zeros(length(t),length(x),length(w));
wave=zeros(length(t),length(x));

for i=1:length(t)
    for j=1:length(x)

        sinx(j,:)=amp.*sin(w*t(i)+K*x(j)+phix);
        sinmult(i,j,:)=sinx(j,:);
      
    end
    for l=1:length(w)
        wave(i,:)=wave(i,:)+sinmult(i,:,l);
    end
    
end
figure
hold on
xlim([0 50]) %[m]
zlim([-30 30]) %[m]
for v=1:length(t)
    wavemat(:)=wave(v,:);
    p=plot(x,wavemat)

    pause(dt)
    delete(p)
end