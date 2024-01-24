syms Q1 Q2 Q3 Q4 Q5 Q1t Q2t Q3t Q4t Q5t real
matrices

Mc = vpa( ditch_small_terms(vpa(M),1e-3),6)

Mi = vpa( inv(Mc),6)
Mi = ditch_small_terms(Mi,1e-3)
Mis = simplify(Mi)

ditch_small_terms(vpa(Mis),1e-1)
Mi(1,1)

Hc = ditch_small_terms(vpa(H),1e-5)

Hc*Mcs