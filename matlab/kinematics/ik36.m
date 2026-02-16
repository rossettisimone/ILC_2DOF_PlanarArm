function q = ik36(R36)
% 'q' sono gli angoli e T è la matrice cinematica diretta 
%  T = [ n s a p;
%        0 0 0 1]
% q03 = [teta1 teta2 teta3] 
%  q = [teta4 teta5 teta6]     

%matrice inversa del braccio antropomorfo
% R03=[cos(teta1)*cos(teta2+teta3) -cos(teta1)*sin(teta2+teta3) sin(teta1);
%     sin(teta1)*cos(teta2+teta3) -sin(teta1)*sin(teta2+teta3) -cos(teta1);
%     sin(teta2+teta3) cos(teta2+teta3) 0];

%matrice inversa delbraccio antropomorfo MODIFICATA (differisce dalla
%definizione per la matrice A23 del braccio)
%matrice del polso sferico
%  R01=[cos(teta1) 0 sin(teta1);
%         sin(teta1) 0 -cos(teta1);
%         0 1 0];
%     R12=[cos(teta2) -sin(teta2) 0;
%         sin(teta2) cos(teta2) 0;
%         0 0 1];
%     R23=[cos(teta3) 0 sin(teta3);
%         sin(teta3) 0 -cos(teta3);
%         0 0 1];
%     R03=R01*R12*R23;

% cinematica inversa ZYZ
teta4=atan2(R36(2,3),R36(1,3));
teta5=atan2(sqrt((R36(1,3))^2+(R36(2,3))^2),R36(3,3));
teta6=atan2(R36(3,2),R36(3,1));

q=[teta4 teta5 teta6]';
end
