%Robot Manipulador/Taller de robotica 2022
%grupo G2
%Autores: 
%Daniel Hernandez
%David Redondo
%Christian Salán
clear
syms d1 theta2 theta3 theta4 theta5
alphaa = [pi/2,pi/2,0,pi/2,0]; % this is the alpha value for all  the link
a=[0, 0, 1,0,0]; % Length of the Link
d=[d1,2,0,0,2]; %Offset
thethaa=[pi/2,pi/2+theta2,theta3,pi/2+theta4,theta5]; % joint angle variation

%Comprobamos la dimensionalidad de los vectores alphaa, a, d y thethaa
n = length(thethaa);
assert((length(alphaa)==n) && (length(a)==n) && (length(d)==n), ...
    'Los vectores no tienen la misma dimensión')

for i = 1:n
     MTH.(sprintf('A%d%d', i-1,i))= ...
        [cos(thethaa(i)),-cos(alphaa(i))*sin(thethaa(i)),sin(alphaa(i))*sin(thethaa(i)),a(i)*cos(thethaa(i));...
        sin(thethaa(i)),cos(alphaa(i))*cos(thethaa(i)),-sin(alphaa(i))*cos(thethaa(i)),a(i)*sin(thethaa(i));...
        0,sin(alphaa(i)),cos(alphaa(i)),d(i);...
        0,0,0,1];
end

for i = 1:n
     disp (sprintf('A%d%d = ', i-1,i))
     disp (MTH.(sprintf('A%d%d', i-1,i)))
end

T = eye (4);
for i=1:n
    T= T*MTH.(sprintf('A%d%d', i-1,i));
end
simplify (T)