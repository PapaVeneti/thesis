syms Q1  Q2  Q3  Q4  Q5 real
syms Q1t Q2t Q3t Q4t Q5t real

%% 
Md = ditch_small_terms(vpa(M),1e-3);
Cd = ditch_small_terms(vpa(C),1e-3);
Gd = ditch_small_terms(vpa(G),1e-3);

termsM = children(M);
termsC = children(C);
termsG = children(G);

termsMd = children(Md);
termsCd = children(Cd);
termsGd = children(Gd);

ntermsM  = 0; ntermsC  = 0; ntermsG  = 0;
ntermsMd = 0; ntermsCd = 0; ntermsGd = 0;
for i = 1:5
    for j=1:5
        ntermsM  = ntermsM + length(termsM {i,j});
        ntermsMd = ntermsMd+ length(termsMd{i,j});

        ntermsC  = ntermsC + length(termsC {i,j});
        ntermsCd = ntermsCd+ length(termsCd{i,j});
    end
ntermsG  = ntermsG + length(termsG {i});
ntermsGd = ntermsGd+ length(termsGd{i});
end

%% 

import casadi.*
%state:
q  = SX.sym('q' ,5) ; 
qt = SX.sym('qt',5) ; 

Q1 = q(1);Q1t = qt(1);
Q2 = q(2);Q2t = qt(2);
Q3 = q(3);Q3t = qt(3);
Q4 = q(4);Q4t = qt(4);
Q5 = q(5);Q5t = qt(5);

Mi = inv(M);
AA = (H*Mi*H');

Nc = eye(5) - Mi*H'*AA*H ;

M = SX.sym('M',5,5)
C = SX.sym('C',5,5)
G = SX.sym('G',5,1)
H = SX.sym('H',2,5)
Ht = SX.sym('Ht',2,5)


