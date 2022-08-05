%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán 
function configuraciones = moveC_conf(robot,q1,q2,q3,q4,q5,x2,y2,z2,x3,y3,z3)

    ik = inverseKinematics('RigidBodyTree',robot);
    weights = [0 0 0 1 1 1]';
    ikInitGuess =[q1;q2;q3;q4;q5];
    endEffector = 'tool';

    
    %Puntos que hace
%     p1=[0.179 0 0.35];p2=[0.23 -0.1 0.4];p3=[0.179 -0.2 0.35];
%     p1=[0.179 0 0.35];p2=[0.179 0.1 0.4];p3=[0.179 0.2 0.35];
%     p1=[0.179 0 0.35];p2=[0.2 0 0.4];p3=[0.3 0 0.35];

%   Aplicamos cinemática directa al punto inicial y lo guardamos en un
%   vector como los puntos intermedio y final.
    MTH_Inicial = getMTH_TCP(robot,q1,q2,q3,q4,q5);
    p1 = [MTH_Inicial(1,4) MTH_Inicial(2,4) MTH_Inicial(3,4)];
    p2 = [x2 y2 z2];
    p3 = [x3 y3 z3];
    
% Utilizamos la función creada por Johannes Korsawe que calcula la
    % circunferencia generada por 3 puntos en el espacio cartesiano. Con
    % ella podemos obtener el centro, radio de la circunferencia y vectores
    % de los puntos al centro.
    [center,rad,v1,v2] = circlefit3d(p1,p2,p3);
    
%     POR SI SE QUIEREN GRAFICAR LOS PUNTOS

%     plot3(p1(:,1),p1(:,2),p1(:,3),'bo');
%     hold on;
%     plot3(p2(:,1),p2(:,2),p2(:,3),'bo');
%     plot3(p3(:,1),p3(:,2),p3(:,3),'bo');

%   Definimos una variable llamada sentido para diferenciar cuando el
%   movimiento que debe realizar el robot es en sentido horario o
%   antihorario.
    sentido = p1-p2;

%   Establecemos los parámetros iniciales de la función
    contador = 1;
    salir = 0;
    x=0;
    y=0;
    z=0;

    %Intervalo en grados, a mayor intervalo -> menos puntos -> menos coste
    %computacional
    intervalo=20;

    if sentido(2) > 0
        for i=1:intervalo:361
	% Este primer bucle for es el encargado de calcular cuantos
            % puntos van a ser necesarios para llevar a cabo la trayectoria
            % circular
            a = i/180*pi;
            x = center(:,1)+sin(a)*rad.*v1(:,1)+cos(a)*rad.*v2(:,1);
            y = center(:,2)+sin(a)*rad.*v1(:,2)+cos(a)*rad.*v2(:,2);
            z = center(:,3)+sin(a)*rad.*v1(:,3)+cos(a)*rad.*v2(:,3);
            if (x >= p3(1)) && (y >= p3(2)) && (z >= p3(3)) && (salir ==0)
                contador = contador+1;
                %plot3(x,y,z,'r.');
            end
            if (x == p1(1)) && (y == p1(2)) && (z == p1(3)) && (salir ==0)
                %plot3(x,y,z,'r.');
                salir = 1;
            end
        end
        %   Creamos una matriz de ceros para contener los puntos de la
        %   trayectoria
        vec_config = zeros(contador,5);
        longitud = contador;
 %Volvemos a establecer la variable contador a cero
        contador = 0;
        angulo=0;

        for i=1:intervalo:361
            a = i/180*pi;
            x = center(:,1)+sin(a)*rad.*v1(:,1)+cos(a)*rad.*v2(:,1);
            y = center(:,2)+sin(a)*rad.*v1(:,2)+cos(a)*rad.*v2(:,2);
            z = center(:,3)+sin(a)*rad.*v1(:,3)+cos(a)*rad.*v2(:,3);
            if (x >= p3(1)) && (y >= p3(2)) && (z >= p3(3)) && (salir ==0)
		%   Generamos una MTH con unicamente una traslación al
                %   punto
                MTH_TCP = trvec2tform ([x y z]);
		%   Aplicamos cinemática inversa a cada uno de los puntos y
                %   los almacemamos en el vector de ceros creado
                %   anteriormente
                qSol = ik(endEffector, MTH_TCP,weights,ikInitGuess);
                ikInitGuess = qSol;
                vec_config(longitud-contador,:)=qSol;
                contador = contador+1;
            end
            if (x == p1(1)) && (y == p1(2)) && (z == p1(3)) && (salir ==0)
                
                MTH_TCP = trvec2tform ([x y z])*axang2tform ([0 0 1 angulo]);
                qSol = ik(endEffector, MTH_TCP,weights,ikInitGuess);
                ikInitGuess = qSol;
                vec_config(longitud-contador,:)=qSol;
                salir = 1;
            end
        end
    else
        for i=1:intervalo:361
            a = i/180*pi;
            x = center(:,1)+sin(a)*rad.*v1(:,1)+cos(a)*rad.*v2(:,1);
            y = center(:,2)+sin(a)*rad.*v1(:,2)+cos(a)*rad.*v2(:,2);
            z = center(:,3)+sin(a)*rad.*v1(:,3)+cos(a)*rad.*v2(:,3);

            if (x >= p1(1)) && (y >= p1(2)) && (z >= p1(3)) && (salir ==0)
                contador = contador+1;
%                plot3(x,y,z,'r.');
            end
            if (x == p3(1)) && (y == p3(2)) && (z == p3(3)) && (salir ==0)
%                 plot3(x,y,z,'r.');
%                 contador = contador+1;
                salir = 1;
            end
        end
        vec_config = zeros(contador,5);
        contador = 1;
        angulo=0;

        for i=1:intervalo:361
            a = i/180*pi;
            x = center(:,1)+sin(a)*rad.*v1(:,1)+cos(a)*rad.*v2(:,1);
            y = center(:,2)+sin(a)*rad.*v1(:,2)+cos(a)*rad.*v2(:,2);
            z = center(:,3)+sin(a)*rad.*v1(:,3)+cos(a)*rad.*v2(:,3);

            if (x >= p1(1)) && (y >= p1(2)) && (z >= p1(3)) && (salir ==0)
                MTH_TCP = trvec2tform ([x y z])*axang2tform ([0 0 1 angulo]);
                qSol = ik(endEffector, MTH_TCP,weights,ikInitGuess);
                ikInitGuess = qSol;
                vec_config(contador,:)=qSol;
                contador = contador+1;
            end
            if (x == p3(1)) && (y == p3(2)) && (z == p3(3)) && (salir ==0)
                MTH_TCP = trvec2tform ([x y z])*axang2tform ([0 0 1 angulo]);
                qSol = ik(endEffector, MTH_TCP,weights,ikInitGuess);
                ikInitGuess = qSol;
                vec_config(contador,:)=qSol;
                salir = 1;
            end
        end
    end
%     grid on;rotate3d on;
    configuraciones = vec_config
end