%%__CALCOLO CINEMATICA INVERSA__%%
function q = ik06 (T06)
% 'q' sono gli angoli e T è la matrice cinematica diretta
%  T = [ n s a p;
%        0 0 0 1]
%  q = [tetha1 tetha2 tetha3]

% parametri Denavit-Hartenberg
a2=1;
a3=1;
d1=0.5;
d6=0.2;
d4=0;
%intera stuttura
n06=T06(1:3,1);
s06=T06(1:3,2);
a06=T06(1:3,3);

%matrici di rotazione
Re=[n06 s06 a06];
%pe posizione dell'endEffector
Pe=T06(1:3,4);
%pw posizione del Frame3 finale del braccio antropomorfo (sottraggo la
%lunghezza del polso e l'altezza del primo frame)
pw_x=Pe(1)+(d6+d4)*a06(1);
pw_y=Pe(2)+(d6+d4)*a06(2);
pw_z=Pe(3)+(d6+d4)*a06(3)-d1;


%calcolo angoli
c3=(pw_x^2+pw_y^2+pw_z^2-(a2^2)-(a3^2))/(2*a2*a3);  % cos(teta3)
if (c3 >= -1 && c3 <= 1)
    s3=-sqrt(1-c3^2);        % sin(teta3)
    teta3=atan2(s3,c3);
    %cos(teta2)=(sqrt(pw_x^2+pw_y^2)*(a2+a3*c3)+pw_z*a3*s3)/(a2^2+a3^2+2*a2*a3*c3);
    %sin(teta2)=(pw_z*(a2+a3*c3)-sqrt(pw_x^2+pw_y^2)*a3*s3)/(a2^2+a3^2+2*a2*a3*c3);
    teta2=atan2((a2+a3*c3)*pw_z-a3*s3*sqrt(pw_x^2+pw_y^2),(a2+a3*c3)*sqrt(pw_x^2+pw_y^2)+a3*s3*pw_z);
    teta1=atan2(pw_y,pw_x);
    
else
    teta3=0;
    teta2=0;
    teta1=atan2(pw_y,pw_x);
end
%matrice del polso sferico
R01=[cos(teta1) 0 sin(teta1);
    sin(teta1) 0 -cos(teta1);
    0 1 0];
R12=[cos(teta2) -sin(teta2) 0;
    sin(teta2) cos(teta2) 0;
    0 0 1];
R23=[cos(teta3+pi) 0 sin(teta3+pi) ;
    sin(teta3+pi) 0 -cos(teta3+pi);
    0  1  0];
R03=R01*R12*R23;
RT=transpose(R03);
R6_3=RT*Re;

% cinematica inversa ZYZ
teta4=atan2(R6_3(2,3),R6_3(1,3));
teta5=atan2(sqrt((R6_3(1,3))^2+(R6_3(2,3))^2),R6_3(3,3));
teta6=atan2(R6_3(3,2),R6_3(3,1));


q=[teta1 teta2 teta3 teta4 teta5 teta6]';
