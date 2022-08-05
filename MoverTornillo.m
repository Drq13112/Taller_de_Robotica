%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán
function MoverTornillo(current_pos,next_pos,velocidad_motor)

    avance_sinfin=11.3; %mm/vuelta
    factor_correccion=1.36;
    factor_correccion_ida=1.325;
    
    % calculo del tiempo necesario para recorrer la distancia buscada
    next_pos;
    current_pos;
    avance = abs(next_pos-current_pos);
    if avance<=5
        velocidad_motor=250;
    end
    num_vueltas=avance/avance_sinfin;
    vueltas_seg=velocidad_motor/1023;

    tiempo_necesario=double(num_vueltas/vueltas_seg)*factor_correccion;
    
    % inicio del movimiento
    calllib('dynamixel','dxl_write_word',1,8,0);% Seteo motor en modo rueda
    
    if next_pos > current_pos 
        velocidad_motor=velocidad_motor+1023;
        tiempo_necesario=double(num_vueltas/vueltas_seg)*factor_correccion_ida;
        %disp('ida')
    
    end
    calllib('dynamixel','dxl_write_word',1,32,velocidad_motor);%Enciendo motor

    % inicio del temporizador
    tic;
    % avance hasta cumplirse el tiempo
    while(toc < tiempo_necesario )
        pause(0.1);
    end
    calllib('dynamixel','dxl_write_word',1,32,0);%Apago Motor
end