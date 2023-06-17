function [A,B,C,D] = state_space(understeer_vehicle,VG)

Lr = understeer_vehicle(1);
Lf = understeer_vehicle(2);
KLr = understeer_vehicle(3);
KLf = understeer_vehicle(4);
Nr = understeer_vehicle(5);
Nf = understeer_vehicle(6);
M = understeer_vehicle(7);
IG = understeer_vehicle(8);

A11 = -(KLf+KLr)/(M*VG);
A12 = -1 + (KLr*Lr-KLf*Lf)/(M*VG);
A21 = (KLr*Lr-KLf*Lf)/(IG);
A22 = (KLf*Lf^2+KLr*Lr^2)/(IG*VG);

B1 = KLf/(M*VG);
B2 = (KLf*Lf)/IG;

A = [A11,A12;A21,A22];
B = [B1;B2];
C = [0, 1];
D = 0;

end