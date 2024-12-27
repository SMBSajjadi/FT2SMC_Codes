function Q = stateCalculation(K1RK,xOld,uold,Ts,tOld)

     K2RK = Rotor2_Dynamic(tOld+Ts/2, xOld+Ts*K1RK/2, uold);
    
    K3RK = Rotor2_Dynamic(tOld+Ts/2, xOld+Ts*K2RK/2, uold);
    
    K4RK = Rotor2_Dynamic(tOld+Ts, xOld+Ts*K3RK, uold);    
    
    Q = xOld+...
           (Ts/6)*(K1RK + 2*K2RK + 2*K3RK + K4RK);

end