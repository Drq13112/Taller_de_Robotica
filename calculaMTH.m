%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán 
function getMTH = calculaMTH()
x=0;l1=0.1725;l2=0.08898;l3=0.260486;a4=0.0145;a5=0.001853;d3=0.005327;d4=0.006194;

syms q1 q2 q3 q4 q5;

% Matriz Denavit Hartenberg genérica
TDH = [ cos(sym('theta'))       -sin(sym('theta'))*cos(sym('alpha'))      sin(sym('theta'))*sin(sym('alpha'))     sym('a')*cos(sym('theta'));
        sin(sym('theta'))        cos(sym('theta'))*cos(sym('alpha'))     -cos(sym('theta'))*sin(sym('alpha'))     sym('a')*sin(sym('theta'));
               0                 sin(sym('alpha'))                        cos(sym('alpha'))                       sym('d');
               0                                  0                                         0                               1              ];
% 
% % Nuestra matriz D-H
% DHTABLE = [  pi/2            x+sym('q1')  0     pi/2;
%              pi/2+sym('q2')      l1       0     pi/2;
%              pi+sym('q3')        d3      l2     0;
%              sym('q4')-pi/2     -d4      a4     pi/2;
%              pi+sym('q5')        l3     -a5     0];
%          
         
% Nuestra matriz D-H aproximada
DHTABLE = [  pi/2            x+sym('q1')  0     pi/2;
             pi/2+sym('q2')      l1       0     pi/2;
             pi+sym('q3')        0       l2     0;
             sym('q4')-pi/2      0        0     pi/2;
             pi+sym('q5')        l3       0     0];

% Grádos de libertad
GLD=5;

%Matrices de paso para cada valor de i
A = cell(1,GLD);
for i = 1:GLD
    alpha = DHTABLE(i,1);
    d = DHTABLE(i,2);
    a = DHTABLE(i,3);
    theta = DHTABLE(i,4);
    A{i} = subs(TDH);
end

%Declaramos una matriz identidad de dimensiones 4x4 como las matrices de
%paso y que nos servirá para postmultiplicar con cada una de las matrices
%A{i} e ir almacenando la matriz postmultiplicada
T = eye(4);
for i=1:GLD
    T = T*A{i};
end

%Utilizamos la función simplify para simplificar la ecuación final
getMTH = simplify(T);

Jacobiano = jacobian([getMTH(1,4),getMTH(2,4),getMTH(3,4)],[q1,q2,q3,q4,q5]);

% getMTH
Jacobiano


% Determinante del jacobiano para analizar las singularidades

% DetJ=det(Jacobiano);
% Jacobiano_simplificado=simplify(DetJ);
% Jacobiano_simplificado

end

