close all
clear global
%% _CONTROLLO ILC DI UN MANIPOLATORE A DUE BRACCI_ %%
%% === tempo di esecuzione 
T_c = 0.01; % tempo di campionamento
t_f=2;
T = [0:T_c:t_f]';% intervallo totale
t_l = length(T); % luncghezza vettore colonna tempo
%% === generazione segnale riferimento
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

x1_d =  q(:,1); % INGRESSO LINK 1
v1_d = [0; diff(x1_d/T_c)]; % VELOCITA' LINK 1

figure(1)
clf
plot(T, x1_d)
title('SEGNALE DI RIFERIMENTO POSIZIONE LINK 1')
xlabel('tempo [s]')
ylabel('angolo [rad]')
xlim([0 2])
grid
figure(12)
clf
plot(T, v1_d)
title('SEGNALE DI RIFERIMENTO VELOCITÀ LINK 1')
xlabel('tempo [s]')
ylabel('velocità [rad/s]')
xlim([0 2])
grid

x2_d = q(:,2); % INGRESSO LINK 2
v2_d = [0; diff(x2_d/T_c)]; % VELOCITA' LINK 2

figure(2)
clf
plot(T, x2_d)
title('SEGNALE DI RIFERIMENTO POSIZIONE LINK 2')
xlabel('tempo [s]')
ylabel('angolo [rad]')
xlim([0 2])
grid
figure(13)
clf
plot(T, v2_d)
title('SEGNALE DI RIFERIMENTO VELOCITÀ LINK 2')
xlabel('tempo [s]')
ylabel('velocità [rad/s]')
xlim([0 2])
grid

%% segnale di riferimento/controllo iniziale per simulink
% blocco simulink "controllo"
controllo.time = T;
controllo.signals.values = [x1_d,x2_d];
controllo.signals.dimensions =2;
% blocco simulink "rif"
rif.time = T;
rif.signals.values = [x1_d,x2_d];
rif.signals.dimensions =2;

% grafico errore posizione 1
figure(3) 
clf
hold on
grid
% grafico errore posizione 2
figure(4)
clf
hold on
grid
% grafico errore velocità 1
figure(5)
clf
hold on
grid

% grafico errore velocità 2
figure(6)
clf
hold on
grid
% grafico riferimento posizione link 1
figure(7)
clf
p1=plot(x1_d,'--','Color','b','LineWidth',2);

xlim([0 200])

hold on
% grafico di riferimento posizione link 2
figure(8)
clf
p2=plot(x2_d,'--','Color','b','LineWidth',2);
xlim([0 200])

hold on
% grafico di riferimento velocità link 1
figure(9)
clf
p3=plot(v1_d,'--','Color','r','LineWidth',2);
xlim([0 200])

hold on
% grafico di riferimento velocità link 2
figure(10)
clf
p4=plot(v2_d,'--','Color','r','LineWidth',2);
xlim([0 200])

hold on
%% parametri di apprendimento
% parametri per ILC
phi = 0.00001; % guadagno errore posizione
gamma = 0.5; % guadagno errore velocità
% parametri per PD
 Kp1 = 100; 
 Kd1 = 45;  
 Kp2 = 40; 
 Kd2 = 20; 
% inizializzo l'errore massimo di posizione e velocità
% considero valori positivi
max_err1=100;
max_err2=100; 
max_err3=100;
max_err4=100;
% conteggio iterazioni
c=0;
% margine di errore
margine=0.05;
loop=0;
%% iterazioni per la correzione del segnale
 while max_err1+max_err2+max_err3+max_err4>margine && loop==0
    % itero finché il sistema non restituisce una traiettoria accettabile e l'errore è minore di una certo margine
    c=c+1;
    if c==30
        loop=1; 
    end
    sim('controller2');  % esegui simulink
    % memorizzo riferimento
    xd1_old = q(:,1);
    xd2_old = q(:,2);

    %% Algoritmo di Apprendimento
    % errori di posizione rispetto al controllo
    e_pos1 = controllo.signals.values(:,1) - uscita.signals.values(:,1);
    e_pos2 = controllo.signals.values(:,2) - uscita.signals.values(:,2);

    % uscita attuale
    uscita1 = uscita.signals.values(:,1); % pos1
    uscita2 = uscita.signals.values(:,2); % pos2
    uscita3 = uscita.signals.values(:,3); % vel1
    uscita4 = uscita.signals.values(:,4); % vel2
    
    % uscita k-esima iterazione
    if c == 5
        k=c;
        uscita1_k = uscita.signals.values(:,1);
        uscita2_k = uscita.signals.values(:,2);
    end
    
    %% errori di posizione rispetto al segnale di riferimento    
    % errore posizione 1
        epos1 = xd1_old - uscita.signals.values(:,1);
    % errore posizione 2
        epos2 = xd2_old - uscita.signals.values(:,2);
    % errore velocità 1
        e_vel1 = v1_d - uscita.signals.values(:,3);
    % errore velocità 2
        e_vel2 = v2_d - uscita.signals.values(:,4);
    
    % registro i valori massimi di errore
    max_err1 = max(epos1);
    max_err2 = max(epos2);
    max_err3 = max(e_vel1);
    max_err4 = max(e_vel2);
  
    %% uk+1 = uk + phi*ek_posizione + gamma*ek_velocità
    % non viene considerato l'integrale dell'errore
    x1_d = x1_d + phi*e_pos1 + gamma*e_vel1;
    x2_d = x2_d + phi*e_pos2 + gamma*e_vel2;
    
    %% aggiorno il segnale di ingresso
    controllo.signals.values = [x1_d,x2_d];
    
    % posizione 1
    hold on
    figure(7)
    plot(uscita1);
    title({'SEGNALE DI USCITA POSIZIONE LINK 1 (ILC)'; strcat(num2str(c),'° Iterazione')})
    xlabel('tempo [s]');
    ylabel('angolo [rad]')
    grid
    hold on
    % posizione 2
    figure(8) 
    plot(uscita2)
    title({'SEGNALE DI USCITA POSIZIONE LINK 2 (ILC)'; strcat(num2str(c),'° Iterazione')})
    xlabel('tempo [s]');
    ylabel('angolo [rad]')
    grid
    % velocità 1
    hold on
    figure(9)
    plot(uscita3)
    title({'SEGNALE DI USCITA VELOCITÀ LINK 1 (ILC)'; strcat(num2str(c),'° Iterazione')})
    xlabel('tempo [s]');
    ylabel('velocità [rad/s]')
    grid
    hold on
    % velocità 2
    figure(10)
    plot(uscita4)
    title({'SEGNALE DI USCITA VELOCITÀ LINK 2 (ILC)'; strcat(num2str(c),'° Iterazione')})
    xlabel('tempo [s]');
    ylabel('velocità [rad/s]')
    grid
    hold on
    
    % errore posizione 1
    figure(3)
    hold on
    plot(c,max_err1,'x','Color','b')
    title({'ERRORE DI POSIZIONE LINK 1 (ILC)'; strcat(num2str(c),'° Iterazione');strcat('ERRORE MASSIMO: ',num2str(max_err1))})
    xlabel('iterazioni')
    ylabel('errore')
    % errore posizione 2
    figure(4)
    hold on
    plot(c,max_err2,'x','Color','b')
    title({'ERRORE DI POSIZIONE LINK 2 (ILC)'; strcat(num2str(c),'° Iterazione');strcat('ERRORE MASSIMO: ',num2str(max_err2))})
    xlabel('iterazioni')
    ylabel('errore')
    % errore velocità 1
    figure(5)
    hold on
    plot(c,max_err3,'x','Color','r')
    title({'ERRORE DI VELOCITÀ LINK 1 (ILC)'; strcat(num2str(c),'° Iterazione');strcat('ERRORE MASSIMO: ',num2str(max_err3))})
    xlabel('iterazioni')
    ylabel('errore')
    % errore velocità 2
    figure(6)
    hold on
    plot(c,max_err4,'x','Color','r')
    title({'ERRORE DI VELOCITÀ LINK 2 (ILC)'; strcat(num2str(c),'° Iterazione');strcat('ERRORE MASSIMO: ',num2str(max_err4))})
    xlabel('iterazioni')
    ylabel('errore')
 end
legend(p1,{'Desiderato'});
legend(p2,{'Desiderato'});
legend(p3,{'Desiderato'});
legend(p4,{'Desiderato'});
figure(20)
clf
plot(T, x1_d)
hold on 
plot(T, xd1_old)
title('SEGNALE DI INGRESSO (CONTROLLO) POSIZIONE LINK 1')
xlabel('tempo [s]')
ylabel('angolo [rad]')
legend({strcat('Controllo ', num2str(c),'° Iterazione'),'Riferimento'});
xlim([0 2])
grid
hold off

figure(21)
clf
plot(T, x2_d)
hold on
plot(T, xd2_old)
title('SEGNALE DI INGRESSO (CONTROLLO) POSIZIONE LINK 2')
xlabel('tempo [s]')
ylabel('angolo [rad]')
legend({strcat('Controllo ', num2str(c),'° Iterazione'),'Riferimento'});

xlim([0 2])
hold off
grid
figure(22)
clf
plot(T, [0; diff(x1_d/T_c)])
hold on 
plot(T, v1_d)
title('SEGNALE DI INGRESSO (CONTROLLO) VELOCITÀ LINK 1')
xlabel('tempo [s]')
ylabel('velocità [rad/s]')
legend({strcat('Controllo ', num2str(c),'° Iterazione'),'Riferimento'});
xlim([0 2])
grid
hold off

figure(23)
clf
plot(T, [0; diff(x2_d/T_c)])
hold on
plot(T, v2_d)
title('SEGNALE DI INGRESSO (CONTROLLO) VELOCITÀ LINK 2')
xlabel('tempo [s]')
ylabel('velocità [rad/s]')
legend({strcat('Controllo ', num2str(c),'° Iterazione'),'Riferimento'});

xlim([0 2])
hold off
grid
%%  rappresentazione grafica del moto del manipolatore
% dati utili del manipolatore
l1=1; % lunghezza link 1
l2=1; % lunghezza link 2
% traiettoria di riferimento desiderata
x1_r = xd1_old;
x2_r = xd2_old;
% traiettoria k-esima iterazione
x1_k = uscita1_k;
x2_k = uscita2_k;
% traiettoria ultima iterazione
x1_c = uscita1;
x2_c = uscita2;
% posizioni giunti xy che poi saranno rappresentati graficamente
% traiettoria k-esima iterazione
x1k = l1.*cos(x1_k); % X1k
y1k = l1.*sin(x1_k); % Y1k
x2k = l1.*cos(x1_k)+l2.*cos(x1_k+x2_k); % X2k
y2k = l1.*sin(x1_k)+l2.*sin(x1_k+x2_k); % Y2k
% traiettoria finale
x1 = l1.*cos(x1_c); % X1c
y1 = l1.*sin(x1_c); % Y1c
x2 = l1.*cos(x1_c)+l2.*cos(x1_c+x2_c); % X2c
y2 = l1.*sin(x1_c)+l2.*sin(x1_c+x2_c); % Y2c
% traiettoria desiderata
x1d = l1.*cos(x1_r); % X1d
y1d = l1.*sin(x1_r); % Y1d
x2d = l1.*cos(x1_r)+l2.*cos(x1_r+x2_r); % X2d
y2d = l1.*sin(x1_r)+l2.*sin(x1_r+x2_r); % Y2d
% velocità di esecuzione
v = 2;
j = 1:v:length(T);

%% animazione 
figure(11) 
for i=1:length(j)-1
    hold off
    % plot(linea 1,colore,linea2,colore,linea 1,marker)
    % manipolatore
    plot([x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'k',[0 x1(j(i))],[0 y1(j(i))],'k',[0 x1(j(i))],[0 y1(j(i))],'o') % manipolatore uscita
    hold on
    plot([x1d(j(i)) x2d(j(i))],[y1d(j(i)) y2d(j(i))],'b',[0 x1d(j(i))],[0 y1d(j(i))],'b',[0 x1d(j(i))],[0 y1d(j(i))],'o') % manipolatore desiderato
    
    % end-effector
    txt_EE = 'O';
    text(x2(j(i)),y2(j(i)),txt_EE,'Color','k','HorizontalAlignment','center'); %end-effector uscita
    txt_EE = 'O';
    text(x2d(j(i)),y2d(j(i)),txt_EE,'Color','b','HorizontalAlignment','center'); %end-effector desiderato
    
    % grafico
    hold on
    title({'MOTO DEL MANIPOLATORE A 2 BRACCI';' BLU: manipolatore ideale';' NERO: manipolatore controllato con ILC'; strcat('ERRORE TOTALE: ',num2str(max_err1+max_err2+max_err3+max_err4))})
    xlabel({'Asse X [m]'})
    ylabel('Asse Y [m]')
    axis([-(l1) (l1+l2/2) -(l1/2) (l1+l2)]); 
    grid
    hold on
    % traiettorie descritte dall'end-effector
    for m=1:i
        p5= plot(x2d(j(m)),y2d(j(m)),'Color','g','Marker','.');        % desiderata
        p6=plot(x2(j(m)),y2(j(m)),'Color','r','Marker','.');         % finale
        p7=plot(x2k(j(m)),y2k(j(m)),'Color','c','Marker','.');        % k-esima
    end
    legend([p5 p6 p7],{'Ideale', strcat(num2str(c),'° Iterazione') , strcat(num2str(k),'° Iterazione' )});
    M(i)= getframe(gcf); 
end
drawnow;    %cambiamenti in tempo reale
%% video animazione (saved in project assets/ folder when present)
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
assetsDir = fullfile(repoRoot, 'assets');
if ~isfolder(assetsDir), assetsDir = pwd; end
v = VideoWriter(fullfile(assetsDir, 'manipolatoreILC.avi'));
v.FrameRate = 5; % velocità riproduzione 
v.Quality = 100; % qualità
open(v);
writeVideo(v,M); 
close(v);
