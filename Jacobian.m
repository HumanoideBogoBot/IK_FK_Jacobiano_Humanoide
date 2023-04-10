clc
clear all
%% Pierna izquierda
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
T02 = simplify(-1*(T01*T12));
T03 = simplify(-1*(T01*T12*T23));
T04 = (T01*T12*T23*T34);
%T05 = simplify(T04*T45);
%T06 = simplify(T05*T56);

T15 = simplify(T12*simplify(T23*T34*T45));
T16 = simplify(T12*simplify(T23*T34*T45)*T56);      
T05 = simplify(-1*(T01*T15));
T06 = simplify(T01*T16);
r06 = T06(1:3,4);
J = simplify([diff(r06,q1) diff(r06,q2) diff(r06,q3) diff(r06,q4) diff(r06,q5) diff(r06,q6); ...
              T05(1:3,2) T04(1:3,1) T03(1:3,1) T02(1:3,1) T01(1:3,2) [0;0;-1]]);
%% Pierna derecha
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
T02 = simplify(T01*T12);
T03 = simplify(T01*T12*T23);
T04 = simplify(-1*(T01*T12*T23*T34));

T15 = simplify(T12*simplify(T23*T34*T45));
T16 = simplify(T12*simplify(T23*T34*T45)*T56);      
T05 = simplify(-1*(T01*T15));
T06 = simplify(T01*T16);
r06 = T06(1:3,4);
J = simplify([diff(r06,q7) diff(r06,q8) diff(r06,q9) diff(r06,q10) diff(r06,q11) diff(r06,q12); ...
              T05(1:3,2) T04(1:3,1) T03(1:3,1) T02(1:3,1) T01(1:3,2) [0;0;-1]])

