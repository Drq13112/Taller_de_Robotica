%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán 
function getWorkspace(n_puntos, vista)
    l1=0.1725;l2=0.08898;l3=0.260486;a4_=0.0145;a5_=0.001853;d3_=0.005327;d4_=0.006194;
    %Inserting D-H convention parameters
    a1 = 0;    alpha1 = pi/2;   t1 = pi/2;
    a2 = 0;    alpha2 = pi/2;   d2 = l1; 
    a3 = l2;   alpha3 = 0;      d3 = d3_;
    a4 = a4_;  alpha4 = pi/2;   d4= -d4_;
    a5 = -a5_; alpha5 = 0;      d5 = l3;
    % Inserting joint limits for Arms
    d1_min = 0;           d1_max = 0.18;
    t2_min = -30*pi/180; t2_max = 30*pi/180;
    t3_min =   -20*pi/180;  t3_max = 180*pi/180; 
    t4_min = -120*pi/180;  t4_max = 120*pi/180;
    t5_min = -145*pi/180; t5_max = 145*pi/180;
    % Monte Carlo method 
    % sampling size
    N = n_puntos;
    d1 = d1_min + (d1_max-d1_min)*rand(N,1);
    t2 = t2_min + (t2_max-t2_min)*rand(N,1);
    t3 = t3_min + (t3_max-t3_min)*rand(N,1);
    t4 = t4_min + (t4_max-t4_min)*rand(N,1);
    t5 = t5_min + (t5_max-t5_min)*rand(N,1);
    for i = 1:N
    A1 = TransMat(a1,alpha1,d1(i),t1);
    A2 = TransMat(a2,alpha2,d2,t2(i));
    A3 = TransMat(a3,alpha3,d3,t3(i));
    A4 = TransMat(a4,alpha4,d4,t4(i));
    A5 = TransMat(a5,alpha5,d5,t5(i));
    T = A1*A2*A3*A4*A5;
    X=T(1,4);
    Y=T(2,4);
    Z=T(3,4);
    plot3(X,Y,Z,'.')
    grid on;    grid minor;
    hold on;
    end
    
    if vista == 1
        view(2); % top view
        title(' Top view');
        xlabel('x (m)');
        ylabel('y (m)');
    elseif vista == 2
        view([1 0 0]); % y-z plane
        title('Side view, Y-Z');
        ylabel('y (m)');
        zlabel('z (m)');
    else
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        camup([1 0 0])
    end
end
