%Inverse Kinematics
this=FourRPR;
this = setCartesianSpace(this,[2 1 0.5]);
JointSpace = InverseKinematics(this)
this = setJointSpace(this,JointSpace);
J=Jacobian(this);
Visualization(this);
