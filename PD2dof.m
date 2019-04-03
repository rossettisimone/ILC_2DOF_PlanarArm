clc
close all
clear global
%% _CONTROLLO PD DI UN MANIPOLATORE A DUE BRACCI_ %%
%% === tempo di esecuzione 
T_c = 0.01; % tempo di campionamento
t_f=2;
T = [0:T_c:t_f]';% intervallo totale
t_l = length(T); % luncghezza vettore colonna tempo
%% === generazione segnale riferimento per link 1 
% traiettoria circolare
inc=0.5;
r=0.5;
c1 = ones(size(T));
c2 = 0.5*ones(size(T));
z = zeros(size(T));
c=[c1 c2 z];
p_0=[0 1.5 0]';
p_f=p_0;
s_0=0;
s_f=norm(2*pi*r);
a_c=(4*s_f)/t_f^2 + inc;
t_c = t_f/2 - 1/2*sqrt((t_f^2*a_c - 4*(s_f - s_0))/a_c);
s=zeros(size(T));
for i=1 : length(T)
    if (T(i)<=t_c)
        s(i)=s_0 + 1/2*a_c*T(i)^2;
    else
        if (t_c<=T(i) && T(i)<=t_f-t_c)
            s(i)=s_0 + a_c*t_c*(T(i)-t_c/2);
        else
            s(i)=s_f - 1/2*a_c*(t_f-T(i))^2;
        end
    end
end
p_e = [r*cos(s./r) r*sin(s./r) z] + c;
cos2=(p_e(:,1).^2 + p_e(:,2).^2 - ones(size(T)))/2;
for i=1:length(s)
   q(i,:)=ik02(p_e(i,:));
end
q1_0=q(1,1);
q2_0=q(1,2);

figure(15) 
title('Traiettoria Circolare')
xlabel('Asse X [m]')
ylabel('Asse Y [m]')
hold on
plot(p_e(:,1),p_e(:,2))


x1_d =q(:,1);
x2_d = q(:,2); 
v1_d = [0; diff(x1_d/T_c)]; % segnale per la velocità
figure(1) 
clf
plot(T, x1_d)
title('SEGNALE DI RIFERIMENTO POSIZIONE LINK 1') 
xlabel('tempo [s]')
ylabel('posizione [rad]')
xlim([0 2])
grid
figure(12) 
clf
plot(T, v1_d)
title('SEGNALE DI RIFERIMENTO VELOCITA LINK 1') 
xlabel('tempo [s]')
ylabel('veelocità [rad/s]')
xlim([0 2])
grid
%% === generazione segnale riferimento per link 2 
v2_d = [0; diff(x2_d/T_c)]; 

figure(2) 
clf
plot(T, x2_d) 
title('SEGNALE DI RIFERIMENTO POSIZIONE LINK 2') 
xlabel('tempo [s]')
ylabel('posizione [rad]')
xlim([0 2])
grid 
figure(13) 
clf
plot(T, v2_d) 
title('SEGNALE DI RIFERIMENTO VELOCITA LINK 2') 
xlabel('tempo [s]')
ylabel('velocità [rad/s]')
xlim([0 2])
grid 
%% === generazione segnale di riferimento/controllo iniziale per simulink
% ---- blocco di controllo simulink "controllo" 
controllo.time = T;
controllo.signals.values = [x1_d,x2_d];
controllo.signals.dimensions =2; 
% ---- blocco di riferimento simulink "rif" 
rif.time = T;
rif.signals.values = [x1_d,x2_d]; 
rif.signals.dimensions =2; 
% ---- grafico errore posizione 1
figure(3) 
clf
hold on 
grid 
% ---- grafico errore posizione 2
figure(4) 
clf 
hold on
grid
% ---- grafico errore velocità 1
figure(5)
clf
hold on 
grid 
% ---- grafico errore velocità 2 
figure(6)
clf 
hold on 
grid 
% ---- segnale di riferimento posizione per il link 1 
figure(7) 
clf 
p2=plot(x1_d,'--','Color','r','LineWidth',2);
xlim([0 200])
hold on 
% ---- segnale di riferimento velocità per il link 2
figure(9) 
clf 
p6=plot(v1_d,'--','Color','r','LineWidth',2);
xlim([0 200])
hold on 
% ---- segnale di riferimento posizione per il link 2
figure(8)
clf
p3=plot(x2_d,'--','Color','r','LineWidth',2); 
xlim([0 200])
hold on 
% ---- segnale di riferimento velocità per il link 2
figure(10) 
clf 
p7=plot(v2_d,'--','Color','r','LineWidth',2); 

xlim([0 200])
hold on 
%% === parametri PID (I=0) 
 Kp1 = 1000; 
 Kd1 = 42; 
 Kp2 = 1000;
 Kd2 = 8;  
%% === iterazioni, per il controllo tramite PID i=1 
for i=1:1 
    i; 
    sim('controller2');  % esegui simulink  
    % ---- riferimento iniziale, traiettoria desiderata
    xd1_old = q(:,1);
    xd2_old = q(:,2); 
    %% === Controllo dell'Errore con PD 
    % ---- errori di posizione rispetto al controllo 
    e_pos1 = controllo.signals.values(:,1) - uscita.signals.values(:,1); %errore posizione 1 
    e_pos2 = controllo.signals.values(:,2) - uscita.signals.values(:,2); %errore posizione 2
    % ---- uscita ottenuta
    uscita1 = uscita.signals.values(:,1); 
    uscita2 = uscita.signals.values(:,2); 
    uscita3 = uscita.signals.values(:,3); 
    uscita4 = uscita.signals.values(:,4); 
    % ---- errori rispetto al segnale di riferimento
    epos1 = xd1_old - uscita.signals.values(:,1);  %errore posizione 1
    epos2 = xd2_old - uscita.signals.values(:,2);  %errore posizione 2
    max_err1 = max(epos1); % errore massimo posizione link 1 
    max_err2 = max(epos2); % errore massimo posizione link 2
    e_vel1 = v1_d - uscita.signals.values(:,3); %errore velocità 1
    e_vel2 = v2_d - uscita.signals.values(:,4); %errore velocità 2
    max_err3 = max(e_vel1);  % errore massimo velocità link 1 
    max_err4 = max(e_vel2);  % errore massimo velocità link 2
    % ---- grafico segnale di uscita 1
    hold on 
    figure(7)
    plot(uscita1) 
    title({'SEGNALE DI USCITA POSIZIONE LINK 1 (PD)'})
    xlabel('tempo [s]');
    ylabel('posizione [rad]')
    grid 
    hold on 
    % ---- grafico segnale di uscita 2
    figure(8) 
    plot(uscita2)
    title({'SEGNALE DI USCITA POSIZIONE LINK 2 (PD)'}) 
    xlabel('tempo [s]'); 
    ylabel('posizione [rad]') 
    grid
    hold on 
    figure(9)
    plot(uscita3) 
    title({'SEGNALE DI USCITA VELOCITÀ LINK 1 (PD)'})
    xlabel('tempo [s]');
    ylabel('velocità [rad/s]')
    grid 
    hold on 
    % ---- grafico segnale di uscita 2
    figure(10) 
    plot(uscita4)
    title({'SEGNALE DI USCITA VELOCITÀ LINK 2 (PD)'}) 
    xlabel('tempo [s]'); 
    ylabel('velocità [rad/2]') 
    grid
    hold on 
    % ---- valore massimo dell'errore rispetto al tempo 
    % ---- grafico link 1 
    figure(3)
    hold on
    plot(T,max_err1,'.','Color','b')
    title({'ERRORE DI POSIZIONE LINK 1 (PD)';strcat('ERRORE MASSIMO: ',num2str(max_err1))})
    xlabel('tempo [s]') 
    ylabel('errore')
    % ---- grafico link 2
    figure(4) 
    hold on 
    plot(T,max_err2,'.','Color','b')
    title({'ERRORE DI POSIZIONE LINK 2 (PD)';strcat('ERRORE MASSIMO: ',num2str(max_err2))})
    xlabel('tempo [s]')
    ylabel('errore') 
    % ---- grafico link 1 
    figure(5) 
    hold on 
    plot(T,max_err3,'.','Color','r') 
    title({'ERRORE DI VELOCITÀ LINK 1 (PD)';strcat('ERRORE MASSIMO: ',num2str(max_err3))}) 
    xlabel('tempo [s]')
        ylabel('errore')

    % ---- grafico link 2
    figure(6) 
    hold on 
    plot(T,max_err4,'.','Color','r')
    title({'ERRORE DI VELOCITÀ LINK 2 (PD)';strcat('ERRORE MASSIMO: ',num2str(max_err4))})
    xlabel('tempo [s]') 
        ylabel('errore') 

end
legend(p2,{'Desiderato'});
legend(p3,{'Desiderato'});
legend(p6,{'Desiderato'});
legend(p7,{'Desiderato'});
figure(20)
clf
plot(T, controllo.signals.values(:,1))
hold on 
plot(T, xd1_old)
title('SEGNALE DI INGRESSO (CONTROLLO) POSIZIONE LINK 1')
xlabel('tempo [s]')
ylabel('angolo [rad]')
legend({'Controllo PD','Riferimento'});
xlim([0 2])
grid
hold off

figure(21)
clf
plot(T, controllo.signals.values(:,2))
hold on
plot(T, xd2_old)
title('SEGNALE DI INGRESSO (CONTROLLO) POSIZIONE LINK 2')
xlabel('tempo [s]')
ylabel('angolo [rad]')
legend({'Controllo PD','Riferimento'});

xlim([0 2])
hold off
grid
figure(22)
clf
plot(T, [0; diff(controllo.signals.values(:,1)/T_c)])
hold on 
plot(T, v1_d)
title('SEGNALE DI INGRESSO (CONTROLLO) VELOCITÀ LINK 1')
xlabel('tempo [s]')
ylabel('velocità [rad/s]')
legend({'Controllo PD','Riferimento'});
xlim([0 2])
grid
hold off

figure(23)
clf
plot(T, [0; diff(controllo.signals.values(:,2)/T_c)])
hold on
plot(T, v2_d)
title('SEGNALE DI INGRESSO (CONTROLLO) VELOCITÀ LINK 2')
xlabel('tempo [s]')
ylabel('velocità [rad/s]')
legend({'Controllo PD','Riferimento'}); 
xlim([0 2])
hold off
grid
%% === rappresentazione grafica del moto del manipolatore 
% ---- dati utili del manipolatore
l1 = 1; % lunghezza link 1
l2 = 1; % lunghezza link 2 
% ---- traiettoria di riferimento desiderata 
x1_r = xd1_old; 
x2_r = xd2_old; 
% ---- traiettoria ottenuta con il controllo PID
x1_c = uscita1;
x2_c = uscita2;
% ---- posizioni giunti xy che poi saranno rappresentati graficamente 
% ---- traiettoria finale 
x1 = l1.*cos(x1_c); % X1c
y1 = l1.*sin(x1_c); % Y1c
x2 = l1.*cos(x1_c)+l2.*cos(x1_c+x2_c);% X2c 
y2 = l1.*sin(x1_c)+l2.*sin(x1_c+x2_c); % Y2c 
% ---- traiettoria desiderata 
x1d = l1.*cos(x1_r); % X1d
y1d = l1.*sin(x1_r); % Y1d 
x2d = l1.*cos(x1_r)+l2.*cos(x1_r+x2_r); % X2d 
y2d = l1.*sin(x1_r)+l2.*sin(x1_r+x2_r); % Y2d 
% ---- velocità di esecuzione  
v = 2;
j = 1:v:length(T);
% %% === animazione 
% figure(11) 
% for i=1:length(j)-1 
%     hold off 
%     % ---- plot(linea 1,colore,linea2,colore,linea 1,marker) 
%     % ---- manipolatore
%     plot([x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'k',[0 x1(j(i))],[0 y1(j(i))],'k',[0 x1(j(i))],[0 y1(j(i))],'o') % manipolatore uscita 
%     hold on 
%     plot([x1d(j(i)) x2d(j(i))],[y1d(j(i)) y2d(j(i))],'b',[0 x1d(j(i))],[0 y1d(j(i))],'b',[0 x1d(j(i))],[0 y1d(j(i))],'o') % manipolatore desiderato
%     % ---- end-effector
%     txt_EE = 'O';  %end-effector uscita 
%     text(x2(j(i)),y2(j(i)),txt_EE,'Color','k','HorizontalAlignment','center'); 
%     txt_EE = 'O';  %end-effector desiderato 
%     text(x2d(j(i)),y2d(j(i)),txt_EE,'Color','b','HorizontalAlignment','center');
%     % ---- grafico
%     hold on 
%     title({'MOTO DEL MANIPOLATORE A 2 BRACCI';'BLU: manipolatore ideale';'NERO: manipolatore controllato con PD';strcat('ERRORE TOTALE: ',num2str(max_err1+max_err2+max_err3+max_err4))})
%     xlabel({'Asse X [m]'})
%     ylabel('Asse Y [m]')
%     axis([-(l1) (l1+l2/2) -(l1/2) (l1+l2)]); 
%     grid 
%     hold on 
%     % ---- traiettorie descritte dall'end-effector 
%     for m=1:i 
%         p5=plot(x2d(j(m)),y2d(j(m)),'Color','g','Marker','.'); 
%         %traiettoria end-effector desiderata 
%         p6=plot(x2(j(m)),y2(j(m)),'Color','r','Marker','.'); 
%         %traiettoria end-effector ottenuta 
%             legend([p5 p6],{'Ideale', 'Ottenuta'});
% 
%     end
%         M(i)= getframe(gcf);  
%         %cattura dei contenuti della figura
% end 
% drawnow; 
% %% === video animazione 
% v = VideoWriter('manipolatorePID.avi');
% v.FrameRate = 5; % velocità riproduzione 
% v.Quality = 100;% qualità
% open(v);
% writeVideo(v,M); 
% close(v);