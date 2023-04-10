% PROGRAMA RELAIZADO POR 
%   Daniel Fuentes Castro           A01750425
%   Leonardo Gracida Muñoz          A01379812
%   Santiago Ortiz Suzarte          A01750402
%   Ana Patricia Islas Mainou       A01751676

clc
clear all

% SYMBOLOS  
syms q7 q8 q9 q10 q11 q12 L
pi = sym("pi");

% MATRICES DE TRANSFORMACION 
T01 = [Rz(-q12),[0;0;0]; 0 0 0 1]; % MATRIZ DE LA BASE, ES FIJA
T12 = [Ry(q11),[0;0;0]; 0 0 0 1];
T23 = [Rx(q10),[0;0;0]; 0 0 0 1];
T34 = [Rx(q9),[0;0;-L]; 0 0 0 1];
T45 = [Rx(-q8),[0;0;-L]; 0 0 0 1];
T56 = [Ry(-q7),[0;0;0]; 0 0 0 1];

% CINEMATICA DIRECTA 
T16 = simplify(T12*T23*T34*T45*T56);

% CINEMATICA INVERSA
syms r11 r12 r13 r21 r22 r23 r31 r32 r33 Px Py Pz
Td = [r11 r12 r13 Px; r21 r22 r23 Py; r31 r32 r33 Pz; 0 0 0 1]; 

% CON LO SIGUIENTE SE PUEDE ENCONTRAR Q3
Tmp1 = [simplify(T16(1,4)^2 + T16(2,4)^2 + T16(3,4)^2) == Td(1,4)^2 + Td(2,4)^2 + Td(3,4)^2]

% UTILIZANDO PITAGORAS

%       2*L^2*(cos(q9) + 1) == Px^2 + Py^2 + Pz^2

% SIMPLIFICANDO

%       c9 = cos(q9) = (Px^2 + Py^2 + Pz^2 -2*L)/2*L^2

% POR TEOREMA 2

%       q9 = acos((Px^2 + Py^2 + Pz^2 -2*L)/2*L^2)
%%
% DADO QUE NO ES POSIBLE DESPEJAR MAS SE BUSCA OTRA EQUIVALENCIA 
Tmp2  = simplify(inv(T12)*Td);
T26 = simplify(T23*T34*T45*T56);

% DE LO SIGUIENTE ES POSIBLE DESPEJAR Q11
Tmp3 = [Tmp2(1:3,4) == T26(1:3,4)]

%        Px cos(q11) -Pz sin (q11) = 0
%%
% UTILIZANDO EL TEOREMA 4

%        sol1 = q11 = atan2(Px, Pz) 
%        sol2 = q11 = atan2(-Px, -Pz)

% DE LO SIGUIETE SE PUEDE DESPEJAR Q10
Tmp5 = [collect(expand((T26(2,4))),[cos(q10), sin(q10)]) == Tmp2(2,4); ...
        collect(expand((T26(3,4))),[sin(q10), cos(q10)]) == Tmp2(3,4)]
    
%       L*sin(q9)*cos(q10) + (L + L*cos(q9))*sin(q10) == Py
%       L*sin(q9)*sin(q10) + (- L - L*cos(q9))*cos(q10) == Pz*cos(q11) + Px*sin(q11)
%%
% SIMPLIFICANDO LAS EXPRESIONES 

%        -L*sin(q9)*sin(q10) + (L*(1+cos(q9)))*cos(q10) == -Pz*cos(q11) - Px*sin(q11)
%        L*sin(q9)*cos(q10) + L*(1+cos(q9))*sin(q10) == Py

% UTILIZANDO EL TEOREMA 6

% q10 = atan2(Py,   -Pz*cos(q11) - Px*sin(q11)) - atan2(L*sin(q9), L*(cos(q9)+1))

% PARA ENCONTRAR Q1 Y Q2 SE REALIZA POR CONCEPTO => PATA PARALLELA AL PISO SIEMPRE (VER ESQUEMA) 

%        q8  = q9 + q10
%        q7 = q11

% PARA Q6 SE ANALIZA COMO CONTRIBUYE AL MOVIMIENTO DEL ROBOT

%        q12 = alpha (ES LA UNICA ROTACION QUE EXISTE SOBRE Z)
%        TIENDE A CERO PARA QUE SE MANTENGA ALIENADO AL EJE DE REFERENCIA




