function posicion=Leer_posicion_de_motor(ID)
   
    %% Leer posici�n del motor
       valor= int16(calllib('dynamixel','dxl_read_word',ID,36));
       posicion=valor*(300/1024)-150;
%        if valor > 511
%             posicion=valor*(300/1024)+150;
%        else
%             posicion=-valor*(300/1024)+150;
%        end  
       fprintf('posicion del motor (en grados):%4.0f\n',posicion)
       