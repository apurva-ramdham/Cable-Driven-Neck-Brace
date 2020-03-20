classdef SevenCable
   properties
       CableLength
       CableFixture
       ManipulatorFixture
       q
   end
   
   properties (Access=private)
       qdot
       qddot
   end
   
   methods
       %constructor
       function this=SevenCable(CableFixture,ManipulatorFixture)
           if nargin==0
               this.CableFixture=[0 0 1;1 0 1;1 1 1;0 1 1;0.5 0 0;1 1 0;0 1 0];
               this.ManipulatorFixture=[-0.15 -0.1 0.05;0.15 -0.1 0.05;0.15 0.1 0.05;-0.15 0.1 0.05;0.0 -0.1 -0.05;0.15 0.1 -0.05;-0.15 0.1 -0.05];
           else
               this.CableFixture=CableFixture;
               this.ManipulatorFixture=ManipulatorFixture;
           end
       end
       
       function this=setCableLength(this,value)
           this.CableLength=value;
           disp('The cable length has been set as the following matrix:-');
           disp(this.CableLength);
       end
       
       function this=setEndEffectorPosition(this,value)
           this.q=value;
           disp('The position of end effector is set to:-');
           disp(this.q);
       end
       
       function L=InverseKinematics(this)
           l=zeros(1,7);
           c=this.q(1:3);
           phi=this.q(4);
           theta=this.q(5);
           psi=this.q(6);
           Q=[cos(theta)*cos(psi) -cos(theta)*sin(psi) sin(theta);
              cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi) cos(phi)*cos(psi)-sin(phi)*sin(theta)*sin(psi) -sin(phi)*cos(theta);
              sin(phi)*sin(psi)-cos(phi)*sin(theta)*cos(psi) sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi) cos(phi)*cos(theta)];
           for i=1:1:7
               a=this.CableFixture(i,:);
               b=this.ManipulatorFixture(i,:);
               d=Q*b';
               l(1,i)=norm(a-c-d');
           end
           L=l;
       end
       
       function J=Jacobian(this)
           J=zeros(7,6);
           l=zeros(7,3);
           c=this.q(1:3);
           phi=this.q(4);
           theta=this.q(5);
           psi=this.q(6);
           Q=[cos(theta)*cos(psi) -cos(theta)*sin(psi) sin(theta);
              cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi) cos(phi)*cos(psi)-sin(phi)*sin(theta)*sin(psi) -sin(phi)*cos(theta);
              sin(phi)*sin(psi)-cos(phi)*sin(theta)*cos(psi) sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi) cos(phi)*cos(theta)];
           for i=1:1:7
               a=this.CableFixture(i,:);
               b=this.ManipulatorFixture(i,:);
               d=Q*b';
               l(i,1:3)=(a-c-d');
               nv=norm(l(i,1:3));
               J(i,1:3)=l(i,1:3)./nv;
               J(i,4:6)=(cross(d,(l(i,1:3)./nv)'))';
           end
           J=-J;
       end
       function q=ForwardKinematics(this,J,l_prev,q_prev)
           Jfk=pinv(J);
           ldot=this.CableLength-l_prev;
           this.qdot=Jfk*ldot';
           q=q_prev+ (this.qdot)';
           
       end
   end
      
end