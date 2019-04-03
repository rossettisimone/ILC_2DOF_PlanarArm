function q = ik02 (p)
% 'q' sono gli angoli e 'p' è la posizione (x,y)
%  q = [tetha1 tetha2]     

% parametri Denavit-Hartenberg
a1=1;
a2=1; 
x=p(1);
y=p(2);

teta2= - acos((x^2+y^2-a1^2-a2^2)/(2*a1*a2)); %teta2 gotimo alto
alfa=atan2(x, y);
beta=acos((x^2+y^2+a1^2-a2^2)/(2*a1*sqrt(x^2+y^2)));
teta1=alfa+beta; %teta1 gomito alto

q=[teta1 teta2];