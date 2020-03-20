# Cable Driven Neck Brace

## 3 RRR
 PLANAR 3RRR
   Forward Kinematics:
  The central points M1, M2 and M3 can be calculated as follows:
M1x=B1x+r1cos(θ1)
M1y=B1y+r1sin(θ1)
M2x=B2x+r2cos(θ2)
M2y=B2y+r2sin(θ2)
M3x=B3x+r3cos(θ3)
M3y=B3y+r3cos(θ3)
Using equation of Circle, we get
(x1-M1x)2 + (y1-M1y)2 = l12 (1)
(x1+dcosθ-M2x)2+(y+dsinθ-M2y)2=l22 (2)
(x1+dcos(θ+ Φ)-M3x)2+(y+dsin(θ+ Φ)-M3y)2=l32 (3)
  Simplifying equations (2) and (3) and subtracting (1) from them, the system reduces to
  following equations:
G1x1+G2y1+G3=l22-l12 (5)
G4x1+G5y1+G6=l32-l12 (6)
where,
G1=2M1x-2M2x+2dcosθ
G2=2M1y-2M2y+2dsinθ
G3=(M2x2+M2y2)-(M1x2+M1y2)+d2-2d(M2xcosθ + B2ysinθ)
G4=2M1x-2M3x+2dcos(θ+ Φ)
G5=2M1y-2M3y+2dsin(θ+ Φ)
G6=(M3x2+M3y2)-(M1x2+M1y2)+d2-2d(M2xcos(θ+ Φ) + B2ysin(θ+ Φ))
  Solving equations (5) and (6),
  𝑥 𝐺𝐺−1𝑙2−𝑙2−𝐺 [1]=[1 2][2 1 3]
  𝑦1 𝐺4 𝐺5 𝑙2 − 𝑙2 − 𝐺 316
 (7)
Now solving by substituting θ as
  𝜃=tan−1( 2𝑇 ) 1−𝑇2
Hence, we obtain an eighth-degree polynomial in T which is evaluated to obtain the values
of θ. Filtering the set of values obtained from theta, we decompose the set of eight
values of θ to single value of θ.
  Hence from θ , x1 and y1 could be obtained from equation (7). A2 and A3 could be
  obtained as follows:
  x2=x1+dcosθ
  y2=y1+dsinθ
  x3=x1+dcos(θ+ Φ)
  y3=y1+dsin(θ+ Φ)
