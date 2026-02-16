clear all
close all

t_f=2;
t=[0:0.01:t_f]';
inc=0.5;
r=0.5;
c1 = ones(size(t));
c2 = 0.5*ones(size(t));
z = zeros(size(t));
c=[c1 c2 z];
p_0=[0 1.5 0]';
p_f=p_0;
s_0=0;
s_f=norm(2*pi*r);
a_c=(4*s_f)/t_f^2 + inc;
t_c = t_f/2 - 1/2*sqrt((t_f^2*a_c - 4*(s_f - s_0))/a_c);
s=zeros(size(t));
for i=1 : length(t)
    if (t(i)<=t_c)
        s(i)=s_0 + 1/2*a_c*t(i)^2;
    else
        if (t_c<=t(i) && t(i)<=t_f-t_c)
            s(i)=s_0 + a_c*t_c*(t(i)-t_c/2);
        else
            s(i)=s_f - 1/2*a_c*(t_f-t(i))^2;
        end
    end
end
p_e = [r*cos(s./r) r*sin(s./r) z] + c;
cos2=(p_e(:,1).^2 + p_e(:,2).^2 - ones(size(t)))/2;
for i=1:length(s)
   q(i,:)=ik02(p_e(i,:));
end
q1_0=q(1,1);
q2_0=q(1,2);

figure(1)
title('Traiettoria Circolare')
xlabel('x[m]')
ylabel('y[m]')
hold on
plot(p_e(:,1),p_e(:,2))
figure(2)
title('Giunto 1')
xlabel('[s]')
ylabel('[rad]')
hold on
plot(t,q(:,1));
figure(3)
title('Giunto 2')
xlabel('[s]')
ylabel('[rad]')
hold on
plot(t,q(:,2));
