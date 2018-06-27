function Rot_e2i = Rotation_e2i( omega,fi,kapa)

%Written by Mozhdeh Shahbazi
Mw0=[1 0 0;0 cos(omega) sin(omega);0 -sin(omega) cos(omega)];
Mf0=[cos(fi) 0 -sin(fi);0 1 0;sin(fi) 0 cos(fi)];
Mk0=[cos(kapa) sin(kapa) 0;-sin(kapa) cos(kapa) 0;0 0 1];
Rot_e2i =(Mk0*Mf0)*Mw0;

end
