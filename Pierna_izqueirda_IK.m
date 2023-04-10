% PROGRAMA RELAIZADO POR 
%   Daniel Fuentes Castro           A01750425
%   Leonardo Gracida Muñoz          A01379812
%   Santiago Ortiz Suzarte          A01750402
%   Ana Patricia Islas Mainou       A01751676

clc
clear all

% SYMBOLOS  
syms q1 q2 q3 q4 q5 q6 L
pi = sym("pi");

% MATRICES DE TRANSFORMACION 
T01 = [Rz(-q6),[0;0;0]; 0 0 0 1]; % MATRIZ DE LA BASE, ES FIJA
T12 = [Ry(q5),[0;0;0]; 0 0 0 1];
T23 = [Rx(-q4),[0;0;0]; 0 0 0 1];
T34 = [Rx(-q3),[0;0;-L]; 0 0 0 1];
T45 = [Rx(q2),[0;0;-L]; 0 0 0 1];
T56 = [Ry(-q1),[0;0;0]; 0 0 0 1];

% CINEMATICA DIRECTA 
T16 = simplify(T12*T23*T34*T45*T56);
T06 = simplify(T12*T23*T34*T45*T56)

% CINEMATICA INVERSA
syms r11 r12 r13 r21 r22 r23 r31 r32 r33 Px Py Pz
Td = [r11 r12 r13 Px; r21 r22 r23 Py; r31 r32 r33 Pz; 0 0 0 1]; 

% CON LO SIGUIENTE SE PUEDE ENCONTRAR Q3
Tmp1 = [simplify(T16(1,4)^2 + T16(2,4)^2 + T16(3,4)^2) == Td(1,4)^2 + Td(2,4)^2 + Td(3,4)^2];

% UTILIZANDO PITAGORAS

%       2*L^2*(cos(q3) + 1) == Px^2 + Py^2 + Pz^2

% SIMPLIFICANDO

%       c3 = cos(q3) = (Px^2 + Py^2 + Pz^2 -2*L)/2*L^2

% POR TEOREMA 2

%       q3 = acos(c3)

% DADO QUE NO ES POSIBLE DESPEJAR MAS SE BUSCA OTRA EQUIVALENCIA 
Tmp2  = simplify(inv(T12)*Td);
T26 = simplify(T23*T34*T45*T56);

% DE LO SIGUIENTE ES POSIBLE DESPEJAR Q5
Tmp3 = [Tmp2(1:3,4) == T26(1:3,4)];

%        Px cos(q5) -Pz sin (q5) = 0

% UTILIZANDO EL TEOREMA 4

%        sol1 = q5 = atan2(Px, Pz) 
%        sol2 = q5 = atan2(-Px, -Pz)

% DE LO SIGUIETE SE PUEDE DESPEJAR Q4 
Tmp5 = [collect(expand((T26(2,4))),[cos(q4), sin(q4)]) == Tmp2(2,4); ...
        collect(expand((T26(3,4))),[sin(q4), cos(q4)]) == Tmp2(3,4)];
    
%       (-L*sin(q3))*cos(q4) + (- L - L*cos(q3))*sin(q4) == Py
%        L*sin(q3)*sin(q4) + (- L - L*cos(q3))*cos(q4) == Pz*cos(q5) + Px*sin(q5)

% SIMPLIFICANDO LAS EXPRESIONES 

%        L*sin(q3)*cos(q4) + L*(1*cos(q3))*sin(q4) == - Py
%        L*sin(q3)*sin(q4) - L*(1*cos(q3))*cos(q4) == Pz*cos(q5) + Px*sin(q5)

% UTILIZANDO EL TEOREMA 6

% q4 = atan2( Pz*cos(q5) + Px*sin(q5) , -Py) - atan2( - L*(1*cos(q3)), L*sin(q3))

% PARA ENCONTRAR Q1 Y Q2 SE REALIZA POR CONCEPTO => PATA PARALLELA AL PISO SIEMPRE (VER ESQUEMA) 

%        q2  = q3 + q4
%        q1 = q5

% PARA Q6 SE ANALIZA COMO CONTRIBUYE AL MOVIMIENTO DEL ROBOT

%        q6 = alpha (ES LA UNICA ROTACION QUE EXISTE SOBRE Z)
%        TIENDE A CERO PARA QUE SE MANTENGA ALIENADO AL EJE DE REFERENCIA




