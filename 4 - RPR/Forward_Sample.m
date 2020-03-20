%Forward Kinematics
linkPos=[-12.5 -8 -3 -10; 6 -2 -10 -5];
KnownPara=[1 3 6;1.5 1.5 6];
radius=3;
delta=[0.7854 2.35619 3.9270 5.4978];
this=FourRPR(linkPos,KnownPara,radius,delta);
[x, y, psi,J] = ForwardKinematics(this)
this = setJointSpace(this,J);
this = setCartesianSpace(this,[x y psi]);
J=Jacobian(this);
Visualization(this)

