classdef FourRPR
    
    properties
        linkPos %[x1 x2 x3 x4;y1 y2 y3 y4]
        KnownPara %known paramters in the form [index,values] [1 2 3 4] represents revolutes 1 2 3 4 and [5 6 7 8] represent prismatic
        radius %radius of the end effector circle
        delta
    end
    properties (Access=private)
        CartesianSpace % [x y psi]
        JointSpace     % [revolute;prismatic]
    end
    
    methods
        %constructor
        function this=FourRPR(linkPos,KnownPara,radius,delta)
            if nargin==0
                %Initilization
                this.linkPos=[3 -3 -3 3; 5 5 -5 -5];
                this.KnownPara=[1 3 7;-1.9872 1.1544 3.9208];
                this.radius=2;
                this.delta=[0.7854 2.35619 3.9270 5.4978];
            else
                this.linkPos=linkPos;
                this.KnownPara=KnownPara;
                this.radius=radius;
                this.delta=delta;
              
            end 
        end
        
        
        % setter
        function this = setCartesianSpace(this,value)
            this.CartesianSpace = value;   
        end
        
        function this = setJointSpace(this,value)
            this.JointSpace = value;
        end
        
        % getter
        function [CartesianSpace,this] = getCartesianSpace(this)
            if isempty(this.CartesianSpace)
                CartesianSpace=[0 0 0];
                this.CartesianSpace=[0 0 0];
            else
                CartesianSpace= this.CartesianSpace;
            end
        end
        
        function [JointSpace,this] = getJointSpace(this)
            if isempty(this.JointSpace)
                JointSpace=[-1.9872 -1.1544 1.1544 1.9872;3.9208 3.9208 3.9208 3.9208];
                this.JointSpace=JointSpace;
            else
                JointSpace=this.JointSpace;
            end
        end
        
        function [x, y, psi,J] = ForwardKinematics(this)
            x_init =Initilization(this.linkPos,this.KnownPara,this.radius,this.delta);
            n = 0;
            options=optimset('MaxFunEvals',1000,'Display','Off');
            [P,~,exitflag] = fsolve(@(x) Equations(x,this.linkPos,this.radius,this.delta,this.KnownPara),x_init,options);
            while(exitflag<1 && n<11)    
               [P,~,exitflag] = fsolve(@(x) Equations(x,this.linkPos,this.radius,this.delta,this.KnownPara),x_init,options);
               x_init(1:5) = x_init(1:5) + n*0.01;
               x_init(6:7) = x_init(6:7) + 0.1;
               x_init(8) = x_init(8) + n*0.01;
               
               n = n+1;
            end
            J=jointArrange(this.KnownPara,P(1:5));
            for i=1:1:8
                if abs(P(i))<10^-4
                    P(i)=0;
                end
            end
            x=P(6);
            y=P(7);
            psi=P(8);
            
        end
        
        function JointSpace = InverseKinematics(this)
            R=zeros(1,4); P=zeros(1,4);
            for i=1:4
                R(i)=atan2(this.CartesianSpace(2)+this.radius*sin(this.CartesianSpace(3)+this.delta(i))-this.linkPos(2,i),this.CartesianSpace(1)+this.radius*cos(this.CartesianSpace(3)+this.delta(i))-this.linkPos(1,i));
                P(i)=sqrt((this.CartesianSpace(1)+this.radius*cos(this.CartesianSpace(3)+this.delta(i))-this.linkPos(1,i))^2+(this.CartesianSpace(2)+this.radius*sin(this.CartesianSpace(3)+this.delta(i))-this.linkPos(2,i))^2);
            end
            JointSpace=[R;P];
      
        end
        
        function J=Jacobian(this)
            A=zeros(8,3);
            B=zeros(8,8);
            for i=1:1:4
                A(i,:) = [1,0,-this.radius*sin(this.CartesianSpace(3)+this.delta(i))];
                A(i+4,:) = [0,1,this.radius*cos(this.CartesianSpace(3)+this.delta(i))];
                B(i,i) = -this.JointSpace(2,i)*sin(this.JointSpace(1,i));
                B(i+4,i+4) = sin(this.JointSpace(1,i));
                B(i+4,i) = this.JointSpace(2,i)*cos(this.JointSpace(1,i));
                B(i,i+4) = cos(this.JointSpace(1,i));
            end
            
            J = pinv(A)*B;
        end
        
        function Visualization(this)
           axis equal
           hold on
           grid on
           theta=0:pi/50:2*pi;
           xc=this.CartesianSpace(1)+this.radius*cos(theta);
           yc=this.CartesianSpace(2)+this.radius*sin(theta);
           plot(this.CartesianSpace(1),this.CartesianSpace(2),'ob','LineWidth',2)
           plot(xc,yc,'m','LineWidth',2);
           EE=zeros(2,4);
           for i=1:1:4
               EE(1,i)=this.CartesianSpace(1)+this.radius*cos(this.delta(i)+this.CartesianSpace(3));
               EE(2,i)=this.CartesianSpace(2)+this.radius*sin(this.delta(i)+this.CartesianSpace(3));
               plot(EE(1,i),EE(2,i),'og','LineWidth',2)
               plot([EE(1,i) this.CartesianSpace(1)],[EE(2,i) this.CartesianSpace(2)],'k--','LineWidth',0.5);
               plot(this.linkPos(1,i),this.linkPos(2,i),'or','LineWidth',2)
               plot([this.linkPos(1,i) EE(1,i)],[this.linkPos(2,i) EE(2,i)],'b','Linewidth',2)
           end
           
        end
            

         

    end
end       