%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán
clear
close all;

%Longitudes
l1=0.1;   l2=0.1;   l3=0.1;   l4=0.1;   l5=0.1;

%Definición de offsets en los arámetros D-H

theta1 = pi/2;  theta2 = pi/2;  theta3 = 0; theta4 = pi/2;  theta5 = 0;

alpha1 = pi/2;  alpha2 = pi/2;  alpha3 = 0; alpha4 = pi/2;  alpha5 = 0;

a1=0;   a2=0;   a3=l3;  a4=0;   a5=0;   a6=0;

d1=0;   d2=l1+l2;   d3=0;   d4=0;   d5=0;   d6=l4+l5;

joint1Limits = [0, 0.25];
joint2Limits = [0, 5*pi/3];
joint3Limits = [0, 5*pi/3];
joint4Limits = [0, 5*pi/3];

%Instancio mi robot con la funcion rigit
myRobot = rigidBodyTree;

%% Cuerpo 1

body1 = rigidBody('body1'); %Crear un objeto de sólido rígido 
jnt1 = rigidBodyJoint('jnt1','prismatic');  %Crear la articulación. 
jnt1.JointAxis = [0, 0, 1];   %Eje de movimiento de la articulación. 
jnt1.HomePosition = 0;  %Posicíon por defecto 
%Defina la posición relativa del nuevo eslabón 
tform = trvec2tform([0, 0, 0])*axang2tform([0 1 0 pi/2])*axang2tform([0 0 1 pi/2]);          
setFixedTransform(jnt1,tform); 


%Defina el angulo inicial (q0) de la articulación
body1.Joint = jnt1; 
%Agregue el primer cuerpo al árbol. Especifique que lo está adjuntando a la base del árbol. La transformación fija definida anteriormente es de la base (principal) al primer cuerpo.
addBody(myRobot,body1,'base');



%% Cuerpo 2
 
body2 = rigidBody('body2'); 
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.JointAxis = [0, 0, 1];   %Eje de movimiento de la articulación. 
jnt2.HomePosition = 0;  %Defina la posicíon por defecto
tform2 = trvec2tform([0, 0, l1+l2])*axang2tform([1 0 0 pi/2])*axang2tform([0 1 0 pi/2]);  %converts the Cartesian representation of a translation vector, trvec, to the corresponding homogeneous transformation, tform. 
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2; 
addBody(myRobot,body2,'body1'); % Add body2 to body1

% show(myRobot);
% ax=gca;
% ax.CameraPosition = []
% campos

%% Cuerpo 3

body3 = rigidBody('body3'); 
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.JointAxis = [0, 0, 1];   %Eje de movimiento de la articulación. 
jnt3.HomePosition = 0;  %Defina la posicíon por defecto 
tform3 = trvec2tform([l3, 0, 0]);  %converts the Cartesian representation of a translation vector, trvec, to the corresponding homogeneous transformation, tform. 
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3; 
addBody(myRobot,body3,'body2'); % Add body3 to body2


%% Cuerpo 4

body4 = rigidBody('body4'); 
jnt4 = rigidBodyJoint('jnt4','revolute');
jnt4.JointAxis = [0, 0, 1];   %Eje de movimiento de la articulación. 
jnt4.HomePosition = 0;  %Defina la posicíon por defecto 
tform4 = trvec2tform([0, 0, 0])*axang2tform([0 1 0 pi/2])*axang2tform([0 0 1 pi/2]);  %converts the Cartesian representation of a translation vector, trvec, to the corresponding homogeneous transformation, tform. 
setFixedTransform(jnt4,tform4);
body4.Joint = jnt4; 
addBody(myRobot,body4,'body3'); % Add body4 to body3

%% Cuerpo 5

body5 = rigidBody('body5'); 
jnt5 = rigidBodyJoint('jnt5','revolute');
jnt5.JointAxis = [0, 0, 1];   %Eje de movimiento de la articulación. 
jnt5.HomePosition = 0;  %Defina la posicíon por defecto 
tform5 = trvec2tform([0, 0, l4+l5]);  %converts the Cartesian representation of a translation vector, trvec, to the corresponding homogeneous transformation, tform. 
setFixedTransform(jnt5,tform5);
body5.Joint = jnt5; 
addBody(myRobot,body5,'body4'); % Add body5 to body4

show(myRobot);

ax=gca;
% view([135 0])
% showdetails(myRobot)

camup([1 0 0])
axis([-0.1,0.5,-0.5,0.5,-0.5,0.5])

% ax.CameraPosition = [-12.1283, -12.1283, -2.4105];
%view([180 0])

% campos
