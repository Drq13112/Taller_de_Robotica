%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán 
function GirarTCP(current_pos,next_pos,velocidad_motor)
%Funcion que gira el tcp en modo rueda
    current_pos
    next_pos
    correccion_rapida=1.6;
    correccion_lenta=1.9;
    num_vueltas=abs(next_pos-current_pos)/360;
    vueltas_seg=(velocidad_motor)/1023;%1023===114rpm
    % inicio del movimiento
    calllib('dynamixel','dxl_write_word',5,8,0);% Seteo motor en modo rueda
    
    if next_pos < -90 || next_pos > 90
        
        if current_pos >-90 && current_pos <90
        
            %TCP arriba y quiere ir hacia abajo
            num_vueltas_inicio=(abs(current_pos)-90)/360;
            num_vueltas_final=(abs(next_pos)-90)/360;
            vueltas_seg=(velocidad_motor)/1023;%1023===114rpm
            tiempo_inicio=double(num_vueltas_inicio/vueltas_seg)*correccion_rapida;%Zona rapida
            tiempo_final=double(num_vueltas_final/vueltas_seg)*correccion_lenta;%Zona lenta

            tiempo_necesario=tiempo_inicio+tiempo_final;
        
        else
            
            %TCP arriba y quiere ir arriba
            num_vueltas=abs(next_pos-current_pos)/360;
            
            vueltas_seg=(velocidad_motor)/1023;%1023===114rpm
            tiempo_necesario=double(num_vueltas/vueltas_seg)*correccion_rapida;%Zona lenta
        end
    else
        
        if current_pos <-90 && current_pos >90
        
            %TCP abajo y quiere ir hacia arriba
            num_vueltas_inicio=(abs(current_pos)-90)/360;
            num_vueltas_final=(abs(next_pos)-90)/360;
            vueltas_seg=(velocidad_motor)/1023;%1023===114rpm
            tiempo_inicio=double(num_vueltas_inicio/vueltas_seg)*correccion_lenta;%Zona rapida
            tiempo_final=double(num_vueltas_final/vueltas_seg)*correccion_rapida;%Zona lenta

            tiempo_necesario=tiempo_inicio+tiempo_final;
        
        else
            
            %TCP abajo y quiere ir abajo
            num_vueltas=abs(next_pos-current_pos)/360;
            vueltas_seg=(velocidad_motor)/1023;%1023===114rpm
            tiempo_necesario=double(num_vueltas/vueltas_seg)*correccion_lenta;%Zona lenta
        end
        
    end
    
    if next_pos < current_pos 
        velocidad_motor=velocidad_motor+1023;
    end
    calllib('dynamixel','dxl_write_word',5,32,velocidad_motor);%Enciendo motor

    % inicio del temporizador
    tic;
    % avance hasta detectar la pegatina blanca o cumplirse el tiempo
    while(toc < tiempo_necesario )
        pause(0.1);
    end
    calllib('dynamixel','dxl_write_word',5,32,0);%Apago Motor

end
