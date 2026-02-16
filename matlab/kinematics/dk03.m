%%__CALCOLO CINEMATICA DIRETTA__%%
function a = dk03 (teta1,teta2,teta3)

robot = robotics.RigidBodyTree();

dhparams = [0     pi/2   0.5    teta1;
            1      0      0     teta2;
            1     pi/2	  0     teta3];
%creazione e aggiunta degli arti revoluti
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');
body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');

%% creo matrici per ciascun DOF richiamando
A01=DHmatrix(dhparams(1,:));
A12=DHmatrix(dhparams(2,:));
A23=DHmatrix(dhparams(3,:));

%impostazione degli arti secondo le matrici ottenute
setFixedTransform(jnt1,A01);
setFixedTransform(jnt2,A12);
setFixedTransform(jnt3,A23);

%assegnazione degli arti
body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;

%creazione dell'albero genealogico
addBody(robot,body1,'base');
addBody(robot,body2,'body1');
addBody(robot,body3,'body2');

%% cinematica diretta
T03=A01*A12*A23; 
p03=T03(1:3,4);
%pe posizione dell'endEffector
 pe_x=p03(1); 
 pe_y=p03(2);
 pe_z=p03(3);  
a=[pe_x pe_y pe_z]';
