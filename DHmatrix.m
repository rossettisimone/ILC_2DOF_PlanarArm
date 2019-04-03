%%__CALCOLO MATRICI ATTRAVERSO I PARAMETRI DH__%%
function [Mdh]=DHmatrix(DHline)
%restituisce riga per riga la matrice associata

a=DHline(1);
alpha=DHline(2);
d=DHline(3);
theta= DHline(4);

Mdh=[cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha)  -cos(theta)*sin(alpha) a*sin(theta);
    0,sin(alpha),cos(alpha),d;
    0,0,0,1];


end