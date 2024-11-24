function [wave,x] = waterlinefunc(dt,tmax,dx,xmax,Tp,Hs )


dw=0.05;
w=0.2:dw:2;
s=487*(Hs^2)./((Tp^4).*(w.^5)).*exp(-1948./((Tp^4).*(w.^4)));
x=0:dx:xmax;
t=0:dt:tmax;
K=(w.^2)./9.81;
phix=(2*pi)*rand(1,length(w));
amp=sqrt(2*dw*s);
sinx=zeros(length(x),length(w));
sinmult=zeros(length(t),length(x),length(w));
wave=zeros(length(t),length(x));

for i=1:length(t)
    for j=1:length(x)

        sinx(j,:)=amp.*sin(w*t(i)-K*x(j)+phix);
        sinmult(i,j,:)=sinx(j,:);
      
    end
    for l=1:length(w)
        wave(i,:)=wave(i,:)+sinmult(i,:,l);
    end
    
end
end

