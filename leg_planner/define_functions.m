import casadi.*

%Matrices 5x2
A5 = SX.sym('a',5,5);
I5 = SX.eye(5);

finv5  = Function('inv'  ,{A5},{A5\I5});
ftrans5 = Function('trans',{A5},{A5'});

%Matrices 2x2
A2 = SX.sym('a',2,2);
I2 = SX.eye(2);

finv2  = Function('inv'  ,{A2},{A2\I2});


% K  = rand(5)
% Ki = f(K)
% 
% Ki*K > 1e-10