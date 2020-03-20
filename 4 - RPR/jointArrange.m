function JointSpace = jointArrange(KnownPara,X)

revoluteCount = KnownPara(1,:)<=4;
Case = sum(revoluteCount);
V = zeros(1,8);
Val = zeros(1,8);
V(KnownPara(1,:)) = 1;
Val(KnownPara(1,:)) = KnownPara(2,:);
V = reshape(V,[4,2])';
Val = reshape(Val,[4,2])';

% Arrange solution
JointSpace = zeros(2,4);
k = 1;
p = 1;
if (Case == 3)
    for i=1:4
        if(V(1,i)==1)
            JointSpace(1,i)=Val(1,i);
        elseif(V(1,i)==0)
            JointSpace(1,i)=X(1);
        end
        JointSpace(2,i)=X(i+1);
    end
elseif (Case == 2)
    for i=1:4
        if(V(1,i)==1 && V(2,i)==1)
            JointSpace(1,i)=Val(1,i);
            JointSpace(2,i)=Val(2,i);
        elseif(V(1,i)==1 && V(2,i)==0)
            JointSpace(1,i)=Val(1,i);
            JointSpace(2,i)=X(p+2);
            p=p+1;
        elseif(V(1,i)==0 && V(2,i)==1)
            JointSpace(1,i)=X(k);
            JointSpace(2,i)=Val(2,i);
            k=k+1;
        elseif(V(1,i)==0 && V(2,i)==0)
            JointSpace(1,i)=X(k);
            JointSpace(2,i)=X(p+2);
            k=k+1;
            p=p+1;
        end
    end
elseif (Case == 1)
    for i=1:4
        if(V(1,i)==1 && V(2,i)==1)
            JointSpace(1,i)=Val(1,i);
            JointSpace(2,i)=Val(2,i);
        elseif(V(1,i)==1 && V(2,i)==0)
            JointSpace(1,i)=Val(1,i);
            JointSpace(2,i)=X(p+3);
            p=p+1;
        elseif(V(1,i)==0 && V(2,i)==1)
            JointSpace(1,i)=X(k);
            JointSpace(2,i)=Val(2,i);
            k=k+1;
        elseif(V(1,i)==0 && V(2,i)==0)
            JointSpace(1,i)=X(k);
            JointSpace(2,i)=X(p+3);
            k=k+1;
            p=p+1;
        end
    end
elseif (Case == 0)
    for i=1:4
        if(V(2,i)==1)
            JointSpace(2,i)=Val(2,i);
        elseif(V(2,i)==0)
            JointSpace(2,i)=X(5);
        end
        JointSpace(1,i)=X(i);
    end
end


end
