function q = ik03 (T03)
% 'q' sono gli angoli e T è la matrice cinematica diretta 
%  T = [ n s a p;
%        0 0 0 1]
%  q = [tetha1 tetha2 tetha3]     

% parametri Denavit-Hartenberg
a2=1;
a3=1; 
d1=1;


%intera stuttura
n03=T03(1:3,1);
s03=T03(1:3,2);
a03=T03(1:3,3);

%matrici di rotazione
Pe=T03(1:3,4);
%pe posizione dell'endEffector
%pw posizione del Frame3 finale del braccio antropomorfo (sottraggo la
%lunghezza del polso
pw_x=Pe(1);
pw_y=Pe(2);
pw_z=Pe(3)-d1;
%pe posizione dell'endEffector


%  teta1=atan2(pw_y,pw_x);
%  
%  teta2=atan2(pw_z*sin(teta1),pw_x);
%  c3= -(cos(teta2)*a03(3))/(sin(teta2)*n03(3));
%  if(c3 >= -1 && c3 <= 1)
%  s3= -sqrt(1-c3^2);
%  teta3=atan2(s3,c3);
%  else
% teta3=0;
%  end

%calcolo angoli
c3=(pw_x^2+pw_y^2+pw_z^2-(a2^2)-(a3^2))/(2*a2*a3);  % cos(teta3)
if (c3 >= -1 && c3 <= 1)
    s3=-sqrt(1-c3^2);        % sin(teta3)
    teta3=atan2(s3,c3);
    c2=(sqrt(pw_x^2+pw_y^2)*(a2+a3*c3)+pw_z*a3*s3)/(a2^2+a3^2+2*a2*a3*c3);      % cos(teta2)
    s2=(pw_z*(a2+a3*c3)-sqrt(pw_x^2+pw_y^2)*a3*s3)/(a2^2+a3^2+2*a2*a3*c3);      % sin(teta2)
    teta2=atan2((a2+a3*c3)*pw_z-a3*s3*sqrt(pw_x^2+pw_y^2),(a2+a3*c3)*sqrt(pw_x^2+pw_y^2)+a3*s3*pw_z);
    teta1=atan2(pw_y,pw_x);

else
    teta1=0;
    teta2=0;
    teta3=0;
end
q=[teta1 teta2 teta3]';
