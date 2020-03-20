classdef PlanarThreeRRR

    properties
        %geometry of device
        sideA
        B1
        B2
        B3
        %link lengths
        Link1
        Link2
        Link3
        %base coordinates
        Ox
        Oy
    end
    
    % properties that obtained through methods, either inputs or outputs
    properties 
        CartesianSpace  % [x y a], a in radians
        JointSpace      % [theta1 theta2 theta3], radians
    end
    
    % methods
    methods
        % constructor
        function this = PlanarThreeRRR(sideA,B1,B2,B3,Link1,Link2,Link3)
            % constructor of the robot
            if nargin==0
                %% INITIALIZATION
%                 Taking the reference point at Origin O(0,0)
                this.sideA=1;    
                this.B1=[0 0];
                this.B2=[4 0];
                this.B3=[2 4];
                this.Link1=[2 1];
                this.Link2=[3 4];
                this.Link3=[3 5];
                this.Ox=0;
                this.Oy=0;
            else
                this.sideA=sideA;
                this.B1=B1;
                this.B2=B2;
                this.B3=B3;
                this.Link1=Link1;
                this.Link2=Link2;
                this.Link3=Link3;
                this.Ox=0;
                this.Oy=0;
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
                CartesianSpace=[2 2 0];
                this.CartesianSpace=[2 2 0];
            else
                CartesianSpace= this.CartesianSpace;
            end
        end
        
        function [JointSpace,this] = getJointSpace(this)
            if isempty(this.JointSpace)
                JointSpace=[0,0,0];
                this.JointSpace=[0,0,0];
            else
                JointSpace=this.JointSpace;
            end
        end
        %% algorithms
        
        function [theta1,theta2,theta3] = InverseKinematics(this)
            % Compute inverse kinematics given the Cartesian space of EE
            % Eight possible solutions, computed using geometric methods
            % input -- this robot with set coordinates of EE [1x3]
            % output -- joint space with all possible configurations
            %           reachable = 0 or 1
            a=this.sideA;
            n=((sin(pi/6))/(sin((2*pi)/3)))*a;
            gamma1=(7*pi)/6;
            gamma2=-pi/6;
            gamma3=pi/2;
            
            bx1 = n*cos(gamma1);
            by1 = n*cos(gamma2);
            bx2 = n*cos(gamma3);
            by2 = n*sin(gamma1);
            bx3 = n*sin(gamma2);
            by3 = n*sin(gamma3);
            
            
            
            l1=this.Link1(1);
            l2=this.Link2(1);
            l3=this.Link3(1);
            
            r1=this.Link1(2);
            r2=this.Link2(2);
            r3=this.Link3(2);
            
            ox=this.Ox;
            oy=this.Oy;
            
            pxb=this.CartesianSpace(1);
            pyb=this.CartesianSpace(2);
            
            phi=this.CartesianSpace(3);
            
            
            
            a1=2*l1*(oy-bx1*sin(phi)-by1*cos(phi)-pyb);
            a2=2*l2*(oy-bx2*sin(phi)-by2*cos(phi)-pyb);
            a3=2*l3*(oy-bx3*sin(phi)-by3*cos(phi)-pyb);
            b1=2*l1*(ox+by1*sin(phi)-bx1*cos(phi)-pxb);
            b2=2*l2*(ox+by2*sin(phi)-bx2*cos(phi)-pxb);
            b3=2*l3*(ox+by3*sin(phi)-bx3*cos(phi)-pxb);
            c1=-(l1^2-2*pyb*oy-2*pxb*ox+bx1^2+by1^2+ox^2+oy^2+pxb^2+pyb^2-r1^2+2*(pxb*bx1+pyb*by1-bx1*ox-by1*oy)*cos(phi)+2*(pyb*bx1-pxb*by1-bx1*oy+by1*ox)*sin(phi));
            c2=-(l2^2-2*pyb*oy-2*pxb*ox+bx2^2+by2^2+ox^2+oy^2+pxb^2+pyb^2-r2^2+2*(pxb*bx2+pyb*by2-bx2*ox-by2*oy)*cos(phi)+2*(pyb*bx2-pxb*by2-bx2*oy+by2*ox)*sin(phi));
            c3=-(l3^2-2*pyb*oy-2*pxb*ox+bx3^2+by3^2+ox^2+oy^2+pxb^2+pyb^2-r3^2+2*(pxb*bx3+pyb*by3-bx3*ox-by3*oy)*cos(phi)+2*(pyb*bx3-pxb*by3-bx3*oy+by3*ox)*sin(phi));
        
            theta1(1)=atan2(a1,b1)-atan2(sqrt(a1^2+b1^2-c1^2),c1);
            theta1(2)=atan2(a1,b1)+atan2(sqrt(a1^2+b1^2-c1^2),c1);
            theta2(1)=atan2(a2,b2)-atan2(sqrt(a2^2+b2^2-c2^2),c2);
            theta2(2)=atan2(a2,b2)+atan2(sqrt(a2^2+b2^2-c2^2),c2);
            theta3(1)=atan2(a3,b3)-atan2(sqrt(a3^2+b3^2-c3^2),c3);
            theta3(2)=atan2(a3,b3)+atan2(sqrt(a3^2+b3^2-c3^2),c3);
        
        end
        
        function [x, y, theta] = ForwardKinematics(this)
            % Compute forward kinematics given base joint angles
            % input -- this robot with set input revolute angles [1x3]
            % output -- universal coordinates of end-effector with
            %           rotation angle theta
            B1x = this.B1(1);
            B1y = this.B1(2);
            B2x = this.B2(1);
            B2y = this.B2(2);
            B3x = this.B3(1);
            B3y = this.B3(2);
            
            M1x = B1x+this.Link1(1)*cos(this.JointSpace(1));
            M1y = B1y+this.Link1(1)*sin(this.JointSpace(1));
            M2x = B2x+this.Link2(1)*cos(this.JointSpace(2));
            M2y = B2y+this.Link2(1)*sin(this.JointSpace(2));
            M3x = B3x+this.Link3(1)*cos(this.JointSpace(3));
            M3y = B3y+this.Link3(1)*sin(this.JointSpace(3));

            l1=this.Link1(2);
            l2=this.Link2(2);
            l3=this.Link3(2);
            
            d1 = this.sideA(1);
            d2 = this.sideA(2);
            d3 = this.sideA(3);
            syms T;
            phi = acos(((d1^2)+(d2^2)-(d3^2))/(2*d1*d2));
      
            spt=sin(phi)*(1+T^2)/(1-T^2)+cos(phi)*2*T/(1+T^2);
            cpt=cos(phi)*(1-T^2)/(1+T^2)-sin(phi)*2*T/(1+T^2);
            G1 = 2*M1x-2*M2x+2*d1*(1-T^2)/(1+T^2);
            G2 = 2*M1y-2*M2y+2*d1*2*T/(1+T^2);
            G3 = (M2x^2)+(M2y^2)-(M1x^2)-(M1y^2)+(d1^2)-(2*d1*(M2x*(1-T^2)/(1+T^2)+M2y*2*T/(1+T^2)));
            G4 = 2*M1x-2*M3x+2*d2*(cpt);
            G5 = 2*M1y-2*M3y+2*d2*(spt);
            G6 = (M3x^2+M3y^2)-(M1x^2+M1y^2)+(d2^2)-2*d2*(M3x*cpt+M3y*spt);
            
            
            %define a matrix 
            G = [G1 G2;G4 G5];
            F = [l2^2-l1^2-G3;l3^2-l1^2-G6];
            Gi=inv(G);
            A =(Gi*F);
            xa=A(1,1)
            ya=A(2,1)
            
            H=(x-B1x)^2+(y-B1y)^2-l1^2==0
            Z=double(solve(H,T))
            theta=2*atan(Z);
            x=subs(xa,T,Z);
            
        end
  
       
        function J = Jacobian(this)
            % compute jacobian given a robot configuration.
                    bx1 = this.B1(1);
            by1 = this.B1(2);
            bx2 = this.B2(1);
            by2 = this.B2(2);
            bx3 = this.B3(1);
            by3 = this.B3(2);
            
            l1=this.Link1(1);
            l2=this.Link2(1);
            l3=this.Link3(1);
            
            r1=this.Link1(2);
            r2=this.Link2(2);
            r3=this.Link3(2);
            
            ox=this.Ox;
            oy=this.Oy;
            
            pxb=this.CartesianSpace(1);
            pyb=this.CartesianSpace(2);
            
            phi=this.CartesianSpace(3);
            
            theta1=this.JointSpace(1);
            theta2=this.JointSpace(2);
            theta3=this.JointSpace(3);
            a1=-2*(pxb-ox+bx1*cos(phi)-l1*cos(theta1)-by1*sin(phi));
            a2=-2*(pxb-ox+bx2*cos(phi)-l2*cos(theta2)-by2*sin(phi));
            a3=-2*(pxb-ox+bx3*cos(phi)-l3*cos(theta3)-by3*sin(phi));
        
            b1=-2*(pyb-oy+by1*cos(phi)-l1*sin(theta1)-bx1*sin(phi));
            b2=-2*(pyb-oy+by2*cos(phi)-l2*sin(theta2)-bx2*sin(phi));
            b3=-2*(pyb-oy+by3*cos(phi)-l3*sin(theta3)-bx3*sin(phi));
        
            c1=-2*(l1*by1*cos(phi-theta1)+l1*bx1*sin(phi-theta1)+(cos(phi))*(pyb*bx1-pxb*by1-bx1*oy+by1*ox)+(sin(phi))*(bx1*ox+by1*oy-pxb*bx1-pyb*by1));
            c2=-2*(l2*by2*cos(phi-theta2)+l2*bx2*sin(phi-theta2)+(cos(phi))*(pyb*bx2-pxb*by2-bx2*oy+by2*ox)+(sin(phi))*(bx2*ox+by2*oy-pxb*bx2-pyb*by2));
            c3=-2*(l3*by3*cos(phi-theta3)+l3*bx3*sin(phi-theta3)+(cos(phi))*(pyb*bx3-pxb*by3-bx3*oy+by3*ox)+(sin(phi))*(bx3*ox+by1*oy-pxb*bx3-pyb*by3));
            
            d1=2*(l1*(oy-pyb)*cos(theta1)+l1*(pxb-ox)*sin(theta1)-l1*by1*cos(phi-theta1)-l1*bx1*sin(phi-theta1));
            d2=2*(l2*(oy-pyb)*cos(theta2)+l2*(pxb-ox)*sin(theta2)-l2*by2*cos(phi-theta2)-l2*bx2*sin(phi-theta2));
            d3=2*(l3*(oy-pyb)*cos(theta3)+l3*(pxb-ox)*sin(theta3)-l3*by3*cos(phi-theta3)-l3*bx3*sin(phi-theta3));
            
            A=[a1 b1 c1;a2 b2 c2;a3 b3 c3];
            D=[d1 0 0;0 d2 0;0 0 d3];
            J=A\D;
        end
                
         
        %% plots
        function axes(~)
            xlabel('X axis','interpreter','latex','FontSize',18);
            ylabel('Y axis','interpreter','latex','FontSize',18);
            axis equal
        end
        function plotRobotEndEffector(this)
            % plot the end-effector of the robot
            dy=sqrt((this.sideA)^2+(this.sideA/2)^2);
            x=this.CartesianSpace(1);
            y=this.CartesianSpace(2);
            x1=x-0.5;
            y1=y-(dy/3);
            x2=x+0.5;
            y2=y-(dy/3);
            x3=x;
            y3=y+2*(dy/3);
            xp=(x1+x2+x3)/3;
            yp=(y1+y2+y3)/3;
            
            hold on
            grid on
            plot(x,y,'k.','markersize',10); 
%             plot(x1,y1,'k.','markersize',10); 
%             plot(x2,y2,'k.','markersize',10); 
%             plot(x3,y3,'k.','markersize',10); 
            plot(xp,yp,'k.','markersize',10);
%             
%             LA1A2 = [x1, x2; y1, y2];
%             line(LA1A2(1,:),LA1A2(2,:),'linewidth',2);
%             LA2A3 = [x2, x3; y2, y3];
%             line(LA2A3(1,:),LA2A3(2,:),'linewidth',2);
%             LA3A1 = [x3, x1; y3, y1];
%             line(LA3A1(1,:),LA3A1(2,:),'linewidth',2);
            
            alpha=this.CartesianSpace(3);
            A_Matrix=[cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
            X1=[x1-xp y1-yp]*A_Matrix;
            X2=[x2-xp y2-yp]*A_Matrix;
            X3=[x3-xp y3-yp]*A_Matrix;
            
            x1a=X1(1)+xp;
            y1a=X1(2)+yp;
            x2a=X2(1)+xp;
            y2a=X2(2)+yp;
            x3a=X3(1)+xp;
            y3a=X3(2)+yp;
            
            plot(x1a,y1a,'k.','markersize',10); 
            plot(x2a,y2a,'k.','markersize',10); 
            plot(x3a,y3a,'k.','markersize',10);
            LA1A2 = [x1a, x2a; y1a, y2a];
            line(LA1A2(1,:),LA1A2(2,:),'linewidth',2);
            LA2A3 = [x2a, x3a; y2a, y3a];
            line(LA2A3(1,:),LA2A3(2,:),'linewidth',2);
            LA3A1 = [x3a, x1a; y3a, y1a];
            line(LA3A1(1,:),LA3A1(2,:),'linewidth',2);
            
        end
        
        function plotRobotLinks(this)
            % plot linkages of the robot
            theta1=this.JointSpace(1);
            theta2=this.JointSpace(2);
            theta3=this.JointSpace(3);
            b1x=this.B1(1);
            b2x=this.B2(1);
            b3x=this.B3(1);
            b1y=this.B1(2);
            b2y=this.B2(2);
            b3y=this.B3(2);
            hold on;
            grid on;
            
            plot(b1x,b1y,'k.','markersize',10);
            plot(b2x,b2y,'k.','markersize',10);
            plot(b3x,b3y,'k.','markersize',10);
            
            l1=this.Link1(1);
            l2=this.Link2(1);
            l3=this.Link3(1);
            
            ox=this.Ox;
            oy=this.Oy;
            m1x=(b1x-ox)+l1*cos(theta1)
            m2x=(b2x-ox)+l2*cos(theta2)
            m3x=(b3x-ox)+l3*cos(theta3)
            m1y=(b1y-oy)+l1*sin(theta1)
            m2y=(b2y-oy)+l2*sin(theta2)
            m3y=(b3y-oy)+l3*sin(theta3)
            
            plot(m1x,m1y,'k.','markersize',10);
            plot(m2x,m2y,'k.','markersize',10);
            plot(m3x,m3y,'k.','markersize',10);
            
            Lb1m1 = [b1x, m1x; b1y, m1y];
            line(Lb1m1(1,:),Lb1m1(2,:),'linewidth',2);
            Lb2m2 = [b2x, m2x; b2y, m2y];
            line(Lb2m2(1,:),Lb2m2(2,:),'linewidth',2);
            Lb3m3 = [b3x, m3x; b3y, m3y];
            line(Lb3m3(1,:),Lb3m3(2,:),'linewidth',2);
            
            
            dy=sqrt((this.sideA)^2+(this.sideA/2)^2);
            x=this.CartesianSpace(1);
            y=this.CartesianSpace(2);
            x1=x-0.5;
            y1=y-(dy/3);
            x2=x+0.5;
            y2=y-(dy/3);
            x3=x;
            y3=y+2*(dy/3);
            xp=(x1+x2+x3)/3;
            yp=(y1+y2+y3)/3;
            
            alpha=this.CartesianSpace(3);
            A_Matrix=[cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];
            X1=[x1-xp y1-yp]*A_Matrix;
            X2=[x2-xp y2-yp]*A_Matrix;
            X3=[x3-xp y3-yp]*A_Matrix;
            
            x1a=X1(1)+xp
            y1a=X1(2)+yp
            x2a=X2(1)+xp
            y2a=X2(2)+yp
            x3a=X3(1)+xp
            y3a=X3(2)+yp
            LA1m1 = [x1a, m1x; y1a, m1y];
            line(LA1m1(1,:),LA1m1(2,:),'linewidth',2);
            LA2m2 = [x2a, m2x; y2a, m2y];
            line(LA2m2(1,:),LA2m2(2,:),'linewidth',2);
            LA3m3 = [x3a, m3x; y3a, m3y];
            line(LA3m3(1,:),LA3m3(2,:),'linewidth',2);
            
            
            
            
            
            
            
            
        end
      
        function plotRobot(this)
            % This is a wrapper of the plotting routine. It calls the other
            % plotting functions to draw the entire robot.
        end
    end
end

