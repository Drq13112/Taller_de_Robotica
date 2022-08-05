%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán 
function myRobot=creaRobot_v2()
%Distancias entre cuerpos
l1=0.1725;l2=0.08898;l3=0.260486;a4_=0.0145;a5_=0.001853;d3_=0.005327;d4_=0.006194;

%Limites de las articulaciones
joint1Limits = [0 0.18];
joint2Limits = [-30*pi/180 30*pi/180];
joint3Limits = [-20*pi/180    180*pi/180];
joint4Limits = [-120*pi/180  120*pi/180];
joint5Limits = [-145*pi/180  145*pi/180];
%Instancio mi robot con la funcion rigidBodyTree
myRobot = rigidBodyTree('DataFormat','column');

%% Cuerpo 1
body1 = rigidBody('body1'); %Crear un objeto de sólido rígido 
jnt1 = rigidBodyJoint('jnt1','prismatic');  %Crear la articulación. 
jnt1.JointAxis = [0 0 1];   %Eje de movimiento de la articulación. 
jnt1.HomePosition = 0;  %Posicíon por defecto 
jnt1.PositionLimits = joint1Limits;
%Defina la posición relativa del nuevo eslabón 
tform = trvec2tform([0, 0, 0]);          
setFixedTransform(jnt1,tform);
%Defina el angulo inicial de la articulación
body1.Joint = jnt1; 
%Agregue el primer cuerpo al árbol. Especifique que lo está adjuntando a la base del árbol. La transformación fija definida anteriormente es de la base (principal) al primer cuerpo.
addBody(myRobot,body1,'base');
%% Cuerpo 2 (Auxiliar)
body2 = rigidBody('body2'); 
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.JointAxis = [0 0 1];   %Eje de movimiento de la articulación. 
jnt2.HomePosition = 0;  %Defina la posicíon por defecto
jnt2.PositionLimits = joint2Limits;
tform2 = trvec2tform([0.065, 0, 0])*axang2tform([1 0 0 pi/2])*axang2tform([0 1 0 pi/2]);  %converts the Cartesian representation of a translation vector, trvec, to the corresponding homogeneous transformation, tform. 
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2; 
addBody(myRobot,body2,'body1');
%% Cuerpo 3
body3 = rigidBody('body3'); 
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.JointAxis = [0 0 1];   %Eje de movimiento de la articulación. 
jnt3.HomePosition = 0;  %Defina la posicíon por defecto
jnt3.PositionLimits = joint3Limits;
tform3 = trvec2tform([0, 0, 0.096])*axang2tform([1 0 0 pi/2])*axang2tform([0 1 0 pi/2]);  %converts the Cartesian representation of a translation vector, trvec, to the corresponding homogeneous transformation, tform. 
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3; 
addBody(myRobot,body3,'body2'); % Add body3 to body2
%% Cuerpo 4
body4 = rigidBody('body4'); 
jnt4 = rigidBodyJoint('jnt4','revolute');
jnt4.JointAxis = [0 0 1];   %Eje de movimiento de la articulación. 
jnt4.HomePosition = 0;  %Defina la posicíon por defecto 
jnt4.PositionLimits = joint4Limits;
tform4 = trvec2tform([0.089, 0, 0])*axang2tform([0 0 1 pi])*trvec2tform([0, 0, d3_]);  %converts the Cartesian representation of a translation vector, trvec, to the corresponding homogeneous transformation, tform. 
setFixedTransform(jnt4,tform4);
body4.Joint = jnt4; 
addBody(myRobot,body4,'body3'); % Add body4 to body3
%% Cuerpo 5
body5 = rigidBody('body5'); 
jnt5 = rigidBodyJoint('jnt5','revolute');
jnt5.JointAxis = [0 0 1];   %Eje de movimiento de la articulación. 
jnt5.HomePosition = 0;  %Defina la posicíon por defecto
jnt5.PositionLimits=joint5Limits;
tform5 = trvec2tform([-0.096,-0.0145, -0.0035])*axang2tform([0 1 0 -pi/2])*axang2tform([0 0 1 -pi/2]);  %converts the Cartesian representation of a translation vector, trvec, to the corresponding homogeneous transformation, tform. 
setFixedTransform(jnt5,tform5);
body5.Joint = jnt5; 
addBody(myRobot,body5,'body4'); % Add body5 to body4
%% TCP
body6 = rigidBody('tool'); 
jnt6 = rigidBodyJoint('jnt6','fixed');
tform6 = trvec2tform([0, 0, 0.1646])*axang2tform([0 0 1 pi]);  %converts the Cartesian representation of a translation vector, trvec, to the corresponding homogeneous transformation, tform. 
setFixedTransform(jnt6,tform6);
body6.Joint = jnt6; 
addBody(myRobot,body6,'body5'); % Add tool to body5
%% Elementos visuales
M1 = axang2tform([0 1 0 pi/2])*axang2tform([0 0 1 pi/2]);
M4 = axang2tform([0 1 0 -pi/2])*axang2tform([0 0 1 pi/2]);
addVisual(myRobot.Bodies{1},"Mesh",'Matlab_Brazo1m.stl',M1);
addVisual(myRobot.Bodies{2},"Mesh",'Matlab_Brazo2m.stl');
addVisual(myRobot.Bodies{3},"Mesh",'Matlab_Brazo3m.stl');
addVisual(myRobot.Bodies{4},"Mesh",'Matlab_Brazo4m.stl',M4);
addVisual(myRobot.Bodies{5},"Mesh",'Matlab_Brazo5m.stl');
end