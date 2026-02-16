%%__CALCOLO CINEMATICA DIRETTA__%%
function a = dk06 (teta1,teta2,teta3,teta4,teta5,teta6)

%% creazione di un corpo rigido
robot = robotics.RigidBodyTree();



dhparams = [0     pi/2   0.5    teta1;
    1      0      0     teta2;
    1     pi/2	  0     teta3;
    0    -pi/2	  0     teta4;
    0     pi/2	  0     teta5;
    0      0     0.2    teta6];
%creazione e aggiunta degli arti revoluti
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');
body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','revolute');
%% creo matrici per ciascun DOF richiamando DHmatrix
%funzione per convertire i parametri DH in matrice
A01=DHmatrix(dhparams(1,:));
A12=DHmatrix(dhparams(2,:));
A23=DHmatrix(dhparams(3,:));
A34=DHmatrix(dhparams(4,:));
A45=DHmatrix(dhparams(5,:));
A56=DHmatrix(dhparams(6,:));
%impostazione degli arti secondo le matrici precedenti
setFixedTransform(jnt1,A01);
setFixedTransform(jnt2,A12);
setFixedTransform(jnt3,A23);
setFixedTransform(jnt4,A34);
setFixedTransform(jnt5,A45);
setFixedTransform(jnt6,A56);
%assegnazione degli arti
body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
%creazione dell'albero genealogico
addBody(robot,body1,'base');
addBody(robot,body2,'body1');
addBody(robot,body3,'body2');
addBody(robot,body4,'body3');
addBody(robot,body5,'body4');
addBody(robot,body6,'body5');
%% cinematica diretta
%matrice anthropomorphic arm MODIFICATA
%(differisce dalla deifinizione per la matrice A23)
T03=A01*A12*A23;
%matrice spherical wrist MODIFICATA
%(differisce dalla deifinizione per la matrice A34)
T36=A34*A45*A56;
T06= T03*T36; %matrice diretta robot 4x4
%% parametri endEffector
%a 'APPROCCIO'
%s 'SLITTAMENTO'
%n 'NORMA'
%p 'PUNTO'
%braccio
a06=T06(1:3,3);
p06=T06(1:3,4);
%pe posizione dell'endEffector
pe_x=p06(1);
pe_y=p06(2);
pe_z=p06(3);
%pw posizione del Frame3 finale del braccio antropomorfo
%(sottraggo lunghezza polso e altezza primo frame)
pw_x=pe_x+(dhparams(6,3)+dhparams(4,3))*a06(1);
pw_y=pe_y+(dhparams(6,3)+dhparams(4,3))*a06(2);
pw_z=pe_z+(dhparams(6,3)+dhparams(4,3))*a06(3)-dhparams(1,3);

a=[pw_x pw_y pw_z]';
