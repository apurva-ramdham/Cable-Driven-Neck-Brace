function x_int=Initilization(linkPos,KnownPara,radius,delta)

V = zeros(1,8);
V(KnownPara(1,:)) = 1;
Z=zeros(1,8);
x_int=zeros(1,8);
x_int(6)=(linkPos(1,1)+linkPos(1,2)+linkPos(1,3)+linkPos(1,4))/4;
x_int(7)=(linkPos(2,1)+linkPos(2,2)+linkPos(2,3)+linkPos(2,4))/4;
x_int(8)=0;
for i=1:1:4
    Z(i+4)=sqrt((linkPos(1,i)-(x_int(6)+radius*cos(delta(i))))^2+(linkPos(2,i)-(x_int(7)+radius*sin(delta(i))))^2);
    Z(i)=pi+atan2(linkPos(2,i)-(x_int(7)+radius*sin(delta(i))),linkPos(1,i)-(x_int(6)+radius*cos(delta(i))));
end

k=1;
for i=1:1:8
    if V(i)==0
        x_int(k)=Z(i);
        k=k+1;
    end
end

x_int(6)=(linkPos(1,1)+linkPos(1,2)+linkPos(1,3)+linkPos(1,4))/4;
x_int(7)=(linkPos(2,1)+linkPos(2,2)+linkPos(2,3)+linkPos(2,4))/4;
x_int(8)=0;
end
