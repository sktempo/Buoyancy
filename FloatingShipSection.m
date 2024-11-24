clc
clear all
close all

dt=0.01;
tmax=50;
t=0:dt:tmax; %[sec]
g=9.81; %[m/s^2]
ton=50; %ship mass in tons
m=ton*1000; %[kg]

aprev=[0,0]; %[m/s^2] previous linear acceleration
anow=[0,0]; %[m/s^2] current linear acceleration
v=[0,0]; %[m/s] current linear velocity
alphaprev=0; %[rad/s^2] previous rotational acceleration
alphanow=0; %[rad/s^2] current rotational acceleration
omega=0; % [rad/s] current rotational velocity
theta=0; % [rad]
I=10000000; % [kg*m^2] rotational moment of inertia about center of rotation 

rho=1000; %[kg/m^3]

xmax=100; %waterline from 0 to xmax
Hs=7; % [m] significant wave height
Tp=11; % [s] peak period
[waterline,xvectorwater]=waterlinefunc(dt,tmax,0.1,xmax,Tp,Hs); %[m]

displacedmass=zeros(1,length(t)); %[ton]

 x=[0.0179	0.1061	0.176	0.3158	0.5	0.6842	0.9484	1.2352	1.4338	1.6772	1.9996	2.3445	2.5776	2.8784	3.157	3.4362	3.7718	4.0281	4.4196	4.8339	5.1472	5.517	5.887	6.3024	6.8069	7.234	7.6728	8.0325	8.4263	8.9331	9.4172	9.8906	10.33	10.9162	11.3904	11.8418	12.3613	12.9378	13.266	13.7975	14.2171	14.6472	15.0216	15.5097	15.941	16.2822	16.5892	16.7944	17.0562	17.2842	17.5014	17.7533	18.0609	18.2664	18.4391	18.6681	18.8743	19.0132	19.1522	19.2912	19.4193	19.5115	19.6158	19.6629	19.7322	19.8031	19.8173	19.8203	19.8452	19.8591	19.9528	19.9923	19.9943	0.0179];
 y=[7.00E-05	-0.76	-1.595	-2.221	-2.9064	-3.5918	-4.1775	-4.7531	-5.1501	-5.5767	-5.9832	-6.3994	-6.7366	-7.0537	-7.3509	-7.5884	-7.8158	-8.0833	-8.3602	-8.6072	-8.8148	-9.0122	-9.1897	-9.3372	-9.5739	-9.6816	-9.7495	-9.8376	-9.8957	-9.9235	-9.9713	-9.9595	-9.9677	-9.9555	-9.874	-9.8126	-9.7111	-9.5497	-9.4191	-9.248	-8.9977	-8.827	-8.5968	-8.2667	-7.9766	-7.6869	-7.4371	-7.2175	-6.9778	-6.7283	-6.449	-6.0702	-5.7607	-5.5212	-5.1725	-4.8434	-4.5244	-4.1857	-3.8371	-3.4886	-3.1002	-2.9109	-2.642	-2.4529	-2.2935	-1.985	-1.7164	-1.438	-1.2191	-0.9704	-0.6419	-0.1147	0.0742	7.00E-05];
% x=[0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 2 3 4 5 6 7 8 9 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 0];
% y=[16 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 1 1 1 1 1 1 1 1 1 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 16];
dropheight=10; %initial section drop height
y=y+dropheight*ones(1,length(x));
xtot=zeros(length(t),length(x)); %save coordinates for printing loop
ytot=zeros(length(t),length(y));

G=[(max(x)-min(x))/2,min(y)+5]; %center of gravity
cor=[0,0];
keelCLpoint=[(max(x)-min(x))/2,min(y)-100]; %CL initial lower point
CLabovekeel=[keelCLpoint(1),max(y)+100]; %CL initial high point

% firstangle=pi/10;
% [x,y] = Rotatepoints(x,y,firstangle,G); %initial angle
% CL=[keelCLpoint(1),CLabovekeel(1);keelCLpoint(2),CLabovekeel(2)]; %initial C.L
% [CL(1,:),CL(2,:)]=Rotatepoints(CL(1,:),CL(2,:),firstangle,G);

p0=101325; %[Pa]
p=p0*ones(1,length(y));
force=zeros(1,length(y)-1); 

Awl=0; %Waterline area
Cd=2; %Heave Drag coefficient

cortotalx=zeros(1,length(t)); % print parameters
cortotaly=zeros(1,length(t));
totalCLx=zeros(length(t),2);
totalCLy=zeros(length(t),2);
totalmlinex=zeros(length(t),2);
totalmliney=zeros(length(t),2);

 for i=1:length(t)
     under=false;
     abovewl=true; %checks if ship is completely above waterline
     submergecounter=0; %count for all y, if all of them are underwater
     totalforcex=0; %x component of buoyant force
     totalforcey=0; %y component of buoyant force
     forcex=zeros(1,length(y)-1);
     forcey=zeros(1,length(y)-1);
     armhorizontal=zeros(1,length(y)-1);
     armvertical=zeros(1,length(y)-1);
     torque=0; %total torque in time t about the center of rotation (cross ofwater line & C.L), Clockwise +, Counter CW -.
     xunderwater=[]; %array for coordinates of current underwater volume
     yunderwater=[]; %array for coordinates of current underwater volume
     p=p0*ones(1,length(y));
     force=zeros(1,length(y)-1); 

     for j=1:length(y)
         xwater=x(j); %move ships coordinate to nearest value of x for water coordinates
         [ d, xindex ] = min( abs( xwater-xvectorwater ) ); % find index for nearest x value
         if y(j)<=waterline(i,xindex) 
             p(j)=p(j)+rho*g*abs(waterline(i,xindex)-y(j)); %calc of pressure values along ship section
         end
         
     end
     
     for k=1:length(y)-1
         
         abovewlnow=true;
         
         dx=x(k+1)-x(k);
         dy=y(k+1)-y(k);
         
         %find index of nearest x cooerdient for body on waterline
         xwater=x(k);
         [ d, xindex ] = min( abs( xwater-xvectorwater ) ); 
%checks if y is on surface or underwater and determine waterline
%coordinates for this time
         if y(k)==waterline(i,xindex) && y(k+1)<waterline(i,xindex)
             ywlleft=waterline(i,xindex);
             xwl1=x(k);
             abovewl=false;
             abovewlnow=false;
             submergecounter=submergecounter+1;
         
         
         elseif y(k+1)==waterline(i,xindex) && y(k)<waterline(i,xindex)
             ywlright=waterline(i,xindex);
             xwl2=x(k+1);
             abovewl=false;
             abovewlnow=false;
             submergecounter=submergecounter+1;
         
         
         elseif y(k) > waterline(i,xindex) && y(k+1) < waterline(i,xindex)
             ywl=waterline(i,xindex);
             xwl1=x(k+1)-(abs(waterline(i,xindex)-y(k+1))/dy)*dx;
             displacedmass(i)=displacedmass(i)+(((abs(waterline(i,xindex)-y(k))+abs(waterline(i,xindex)-y(k+1)))/2*abs(xwl1-x(k+1))))*rho/1000; %ton
             abovewl=false;
             abovewlnow=false;
             

         elseif y(k) < waterline(i,xindex) && y(k+1) > waterline(i,xindex)
             ywl=waterline(i,xindex);
             xwl2=x(k)+(abs(waterline(i,xindex)-y(k))/dy)*dx;
             displacedmass(i)=displacedmass(i)+(((abs(waterline(i,xindex)-y(k))+abs(waterline(i,xindex)-y(k+1)))/2*abs(xwl2-x(k))))*rho/1000; %ton;
             abovewl=false;
             abovewlnow=false;
         
         
         elseif y(k)<waterline(i,xindex) && y(k+1)<waterline(i,xindex)
             displacedmass(i)=displacedmass(i)+(((abs(waterline(i,xindex)-y(k))+abs(waterline(i,xindex)-y(k+1)))/2*(dx)))*rho/1000; %ton
             abovewl=false;
             abovewlnow=false;
             submergecounter=submergecounter+1;
         else
             ywlleft=0;
             ywlright=0;
             xwl1=min(x);
             xwl2=max(x);
         end 
         %calculation of buoyant force on each segment of section
         midpoint=[(dx)/2,(dy)/2]+[x(k),y(k)]; %point at which the buoyant force acts
         endpoint = midpoint+[-dy,dx]; %normal end point inwards to the body
         normalvector=endpoint-midpoint;
         unitnormal=normalvector./norm(normalvector); %normazlize the normal to length 1
         
         force(k)=((p(k)+p(k+1))/2)*sqrt((dx)^2+(dy)^2); %buoyant force on segment area
         forcevec=force(k).*unitnormal; %force vec on segment
         
         if abovewlnow ==false
             %current segment underwater - take the force and arm for later
             %calculation of torque due to buoyant force 

             forcex(k)=forcevec(1); %current x force for segment 
             forcey(k)=forcevec(2); %current y force for segment
             armhorizontal(k)=midpoint(2); %save midpoint for later calc of torque due to horizontal force
             armvertical(k)=midpoint(1); %save midpoint for later calc of torque arm due to vertical force
   
         end
         
         totalforcex=totalforcex+forcevec(1); % add up all forces along ship outer section line
         totalforcey=totalforcey+forcevec(2);
            
     end
     
     if abovewl == false 
         %calc of the underwater volume (BASED ON FLAT WATERLINE AT 0)
         %vector for the underwater volume
         under=true;
         xunderwater=xwl1;
         yunderwater=ywlleft;
         for j=1:length(y)
             if y(j)<waterline(i,j) 
                 xunderwater=[xunderwater,x(j)]; 
                 yunderwater=[yunderwater,y(j)];
             end
         end
         xunderwater=[xunderwater,xwl2,xwl1];
         yunderwater=[yunderwater,ywlright,ywlleft];
         
         [bx,by,Area] = xycentroid(xunderwater,yunderwater); %center of buoyancy and its area

     end
     

     if submergecounter == length(y)-1 %Sumberged check and C.O.R calc for that case
         %fully submerged 
          cor=[bx,by]; %center of roll rotation at center of buoyancy
%          cor=G;
         xwl1=min(x);
         xwl2=max(x);
         abovewl=false;
         torque=torque+sum((cor(1)-armvertical).*forcey); %torque due to vertical buoyant force on segment
         torque=torque+sum((armhorizontal-cor(2)).*forcex); %torque due to horizontal buoyant force on segment
         torque=torque+(G(1)-cor(1))*(m*g); %torque due to gravity force (no effect when cor=G)
%          torque=(G(1)-bx)*totalforcey; %torque due to vertical buoyant force on segment
         cortotalx(i)=cor(1);
         cortotaly(i)=cor(2);
     
     elseif abovewl==false && submergecounter ~= length(y)-1 
         %section on surface of water
%           SecondAreaMoment=1*((xwl2-xwl1)^3)/12; %calc for Ixx
%           BM=SecondAreaMoment/Area*1; %Metacentric radius %calc for
%           metacentric radius
%           Mline = [-9999999999999999999999999999999,99999999999999999999999999999999;BM+by,BM+by];
%          cor=InterX(CL,Mline); %center of roll rotation at metacenter
%          height intersection with ships CL
          cor=G; 
%           if under==true
%               
%               cor=[keelCLpoint(1),BM+by];
%           else
%               cor=G;
%           end

         torque=torque+sum((armhorizontal-cor(2)).*forcex); %torque due to horizontal buoyant force on segment
         torque=torque+sum((cor(1)-armvertical).*forcey); %torque due to vertical buoyant force on segment
         torque=torque+(G(1)-cor(1))*(m*g); %torque due to gravity force
     %   torque=(G(1)-bx)*totalforcey; %torque due to vertical buoyant force on segment
         cortotalx(i)=cor(1);
         cortotaly(i)=cor(2);
%          totalCLx(i,:)=CL(1,:);
%          totalCLy(i,:)=CL(2,:);
%          totalmlinex(i,:)=Mline(1,:);
%          totalmliney(i,:)=Mline(2,:);

     end
     
     if  v(2)<0 && abovewl==false %Heave drag calculation
         Awl=abs(xwl2-xwl1)*1;
         drag=0.5*Cd*rho*(v(2)^2)*Awl;
         
     elseif v(2)>0 && abovewl==false
         Awl=abs(xwl2-xwl1)*1;
         drag=-0.5*Cd*rho*(v(2)^2)*Awl;
         
     else
         drag=0;
     end
     
     %Linear acceleration,velocity and positions update
%      anow=[totalforcex/m,((totalforcey+drag)/m)-g];  
     anow=[0/m,((totalforcey+drag)/m)-g];  %acc without x values (ship stationary horizontaly)
     %Updates for location coordinates using Leap-Frog integration method
     x=x+(dt*v(1).*ones(1,length(x)))+(0.5*aprev(1)*(dt^2)).*ones(1,length(x));
     y=y+(dt*v(2).*ones(1,length(y)))+(0.5*aprev(2)*(dt^2)).*ones(1,length(y));
     G=G+(dt.*v)+(0.5*aprev*(dt^2));
     keelCLpoint=keelCLpoint+(dt.*v)+(0.5*aprev*(dt^2));
     CLabovekeel=CLabovekeel+(dt.*v)+(0.5*aprev*(dt^2));
     cor=cor+(dt.*v)+(0.5*aprev*(dt^2));
     v=v+(0.5*(aprev+anow).*dt);  
     aprev=anow;
     
     %Rotational acceleration,velocity and positions update using Leapfrog
     %as well
     if abovewl==false
         
         alphanow=torque/I;
         theta=(dt*omega)+(0.5*alphaprev*(dt^2));
         if theta<0 %CCW rotation 
             
             [x,y]=Rotatepoints(x,y,abs(theta),cor);
             [G(1),G(2)]=Rotatepoints(G(1),G(2),abs(theta),cor);
             [keelCLpoint(1),keelCLpoint(2)]=Rotatepoints(keelCLpoint(1),keelCLpoint(2),abs(theta),cor);
             [CLabovekeel(1),CLabovekeel(2)]=Rotatepoints(CLabovekeel(1),CLabovekeel(2),abs(theta),cor);
%              [CL(1,:),CL(2,:)]=Rotatepoints(CL(1,:),CL(2,:),abs(theta),cor);
         elseif theta>0 %CW rotation
             
             [x,y]=Rotatepoints(x,y,-1*abs(theta),cor); 
             [G(1),G(2)]=Rotatepoints(G(1),G(2),-1*abs(theta),cor);
             [keelCLpoint(1),keelCLpoint(2)]=Rotatepoints(keelCLpoint(1),keelCLpoint(2),-1*abs(theta),cor);
             [CLabovekeel(1),CLabovekeel(2)]=Rotatepoints(CLabovekeel(1),CLabovekeel(2),-1*abs(theta),cor);
%              [CL(1,:),CL(2,:)]=Rotatepoints(CL(1,:),CL(2,:),-1*abs(theta),cor);
         end

         omega=omega+(0.5*(alphaprev+alphanow)*dt);
         alphaprev=alphanow;
     end

     xtot(i,:)=x;
     ytot(i,:)=y;
     
     CL=[keelCLpoint(1),CLabovekeel(1);keelCLpoint(2),CLabovekeel(2)]; %C.L update

 end
 
figure;
hold on;
axis equal
xlim([-10 100]) %[m]
ylim([-30 30]) %[m]
pause('on')

set(gcf,'Renderer','opengl')
for l=1:length(t) %drawing loop
     
     q = hgtransform;
     patch('XData',xtot(l,:),'YData',ytot(l,:),'FaceColor','yellow','Parent',q)
     %q.Matrix=makehgtform('axisrotate',[0,1,1],rad2deg(1));
     plot(cortotalx(l),cortotaly(l),'r*')
     water=plot(xvectorwater,waterline(l,:))
%      l1=plot(totalCLy(l,:),totalCLx(l,:),'-')
%      l2=plot(totalmliney(l,:),totalmlinex(l,:),'-')

     disp (displacedmass(l))
     
     pause(dt)
     delete(q)
     delete(water)

end
