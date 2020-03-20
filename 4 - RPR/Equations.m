function G = Equations(x,linkPos,radius,delta,KnownPara)

% Checking upon the cases for the user input parameters
%       If case=0 indicates 3 prismatic joints
%       If case=1 indicates 1 revolute and 2 prismatic joints
%       If case=2 indicates 2 revolute and 1 prismatic joints
%       If case=3 indicates 3 revolute joints
revoluteCount = KnownPara(1,:)<=4;
Case = sum(revoluteCount);


V = zeros(1,8); 
Val = zeros(1,8);
V(KnownPara(1,:)) = 1;
Val(KnownPara(1,:)) = KnownPara(2,:);
V = reshape(V,[4,2])';
Val = reshape(Val,[4,2])';
% %#ok<AGROW> is used to avoid errors during fsolve
% Construct Equations
k = 1;
p = 1;
if (Case == 3)
    for i=1:4
        if(V(1,i)==1)
            G(i)=linkPos(1,i)+x(i+1)*cos(Val(1,i))-x(6)-radius*cos(x(8)+delta(i));%#ok<AGROW>
            G(i+4)=linkPos(2,i)+x(i+1)*sin(Val(1,i))-x(7)-radius*sin(x(8)+delta(i));%#ok<AGROW>
        elseif(V(1,i)==0)
            G(i)=linkPos(1,i)+x(i+1)*cos(x(1))-x(6)-radius*cos(x(8)+delta(i));%#ok<AGROW>
            G(i+4)=linkPos(2,i)+x(i+1)*sin(x(1))-x(7)-radius*sin(x(8)+delta(i));%#ok<AGROW>
        end
    end
elseif (Case == 2)
    for i=1:4
        if(V(1,i)==1 && V(2,i)==1)
            G(i)=linkPos(1,i)+Val(2,i)*cos(Val(1,i))-x(6)-radius*cos(x(8)+delta(i));%#ok<AGROW>
            G(i+4)=linkPos(2,i)+Val(2,i)*sin(Val(1,i))-x(7)-radius*sin(x(8)+delta(i));%#ok<AGROW>
        elseif(V(1,i)==1 && V(2,i)==0)
            G(i)=linkPos(1,i)+x(p+2)*cos(Val(1,i))-x(6)-radius*cos(x(8)+delta(i));%#ok<AGROW>
            G(i+4)=linkPos(2,i)+x(p+2)*sin(Val(1,i))-x(7)-radius*sin(x(8)+delta(i));%#ok<AGROW>
            p=p+1;
        elseif(V(1,i)==0 && V(2,i)==1)
            G(i)=linkPos(1,i)+Val(2,i)*cos(x(k))-x(6)-radius*cos(x(8)+delta(i));%#ok<AGROW>
            G(i+4)=linkPos(2,i)+Val(2,i)*sin(x(k))-x(7)-radius*sin(x(8)+delta(i));%#ok<AGROW>
            k=k+1;
        elseif(V(1,i)==0 && V(2,i)==0)
            G(i)=linkPos(1,i)+x(p+2)*cos(x(k))-x(6)-radius*cos(x(8)+delta(i));%#ok<AGROW>
            G(i+4)=linkPos(2,i)+x(p+2)*sin(x(k))-x(7)-radius*sin(x(8)+delta(i));%#ok<AGROW>
            k=k+1;
            p=p+1;
        end
    end
elseif (Case == 1)
    for i=1:4
        if(V(1,i)==1 && V(2,i)==1)
            G(i)=linkPos(1,i)+Val(2,i)*cos(Val(1,i))-x(6)-radius*cos(x(8)+delta(i));%#ok<AGROW>
            G(i+4)=linkPos(2,i)+Val(2,i)*sin(Val(1,i))-x(7)-radius*sin(x(8)+delta(i));%#ok<AGROW>
        elseif(V(1,i)==1 && V(2,i)==0)
            G(i)=linkPos(1,i)+x(p+3)*cos(Val(1,i))-x(6)-radius*cos(x(8)+delta(i));%#ok<AGROW>
            G(i+4)=linkPos(2,i)+x(p+3)*sin(Val(1,i))-x(7)-radius*sin(x(8)+delta(i));%#ok<AGROW>
            p=p+1;
        elseif(V(1,i)==0 && V(2,i)==1)
            G(i)=linkPos(1,i)+Val(2,i)*cos(x(k))-x(6)-radius*cos(x(8)+delta(i));%#ok<AGROW>
            G(i+4)=linkPos(2,i)+Val(2,i)*sin(x(k))-x(7)-radius*sin(x(8)+delta(i));%#ok<AGROW>
            k=k+1;
        elseif(V(1,i)==0 && V(2,i)==0)
            G(i)=linkPos(1,i)+x(p+3)*cos(x(k))-x(6)-radius*cos(x(8)+delta(i));%#ok<AGROW>
            G(i+4)=linkPos(2,i)+x(p+3)*sin(x(k))-x(7)-radius*sin(x(8)+delta(i));%#ok<AGROW>
            k=k+1;
            p=p+1;
        end
    end
elseif (Case == 0)
    for i=1:4
        if(V(2,i)==1)
            G(i)=linkPos(1,i)+Val(2,i)*cos(x(i))-x(6)-radius*cos(x(8)+delta(i));%#ok<AGROW>
            G(i+4)=linkPos(2,i)+Val(2,i)*sin(x(i))-x(7)-radius*cos(x(8)+delta(i));%#ok<AGROW>
        elseif(V(2,i)==0)
            G(i)=linkPos(1,i)+x(5)*cos(x(i))-x(6)-radius*cos(x(8)+delta(i));%#ok<AGROW>
            G(i+4)=linkPos(2,i)+x(5)*sin(x(i))-x(7)-radius*sin(x(8)+delta(i));%#ok<AGROW>
        end
    end
end


end