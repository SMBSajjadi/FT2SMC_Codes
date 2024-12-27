function [XDOT,wStar,W,fPhi,fTheta,fPsi] =...
              Rotor2_Dynamic(t,x,u)
    %% A. System Parameters
    
        L = 0.47/2;      % One-Half Length. Full Length equals 2*L = 47cm
        m = 1;             % Mass of the Quadrotor
        g = 9.81;

        Ix = 0.0081;        % X Axis Moment of Intertia
        Iy = Ix;               % Y Axis Moment of Intertia
        Iz = 0.0142;        % Z Axis MOMENT of Intertia
        JTP = 10.4e-5;    
        
        b = 5.42e-5;    % Drag Force Coefficient
        d = 1.1e-6;      % Drag Torque Coefficient
        
        Kf = 1e-6;
        Kt = 1.2e-6;

        %% Control Part
    
        Transform_Matrix = [b       b       b       b
                                        0    -b*L    0       b*L
                                       -b*L   0      b*L    0
                                        d     -d      d       -d];
                                           
        uThrust = m*sqrt(u(1)^2+u(2)^2+(g+u(3))^2);
        Sol_Vector = [uThrust u(4) u(5) u(6)]';        % [Uz, Uphi, Utheta, Upsi]'
        Squared_W = (linsolve(Transform_Matrix,Sol_Vector));
        
        w1s = Squared_W(1);
        w2s = Squared_W(2);
        w3s = Squared_W(3);
        w4s = Squared_W(4);
        
        W = real(sqrt([w1s w2s w3s w4s]));
        w1 = W(1);
        w2 = W(2);
        w3 = W(3);
        w4 = W(4);
                
    %% B. State Vector
    
    xDot = x(2);
    yDot = x(4);
    zDot = x(6);
    
    phiDot = x(8);
    thetaDot = x(10);
    sayDot = x(12);
    
    wStar = (w1 + w3 - w2 - w4);        % Disturbance
       
        %% C. Fault Injection 
            
        ufx = -0.8*sin(0.1*t-3.04)+0.4*sin(0.44*t-13.46)+0.08*sin(pi*t/2);
        ufy = 0.5*sin(0.4*t)+0.5*cos(0.7*t);
        ufz = 0.5*cos(0.4*t);
        
        ufPhi = 0.5*cos(0.4*t);
        ufTheta = 0.5*sin(0.5*t);
        ufSay = 0;
        
        %% Unkonown Disturbance to be Estimated Using RNN
               
        SWITCH = 0;     % 1 for Disturbance. Zero for Lack of Disturbance.

        if(SWITCH == 0)
            ufx = 0;
            ufy = 0;
            ufz = 0;
            ufPhi = 0;
            ufTheta = 0;
            ufSay = 0;
        end

      %% State Space 
       
       xDoubleDot = u(1)-Kf*xDot/m+ufx;
       yDoubleDot = u(2)-Kf*yDot/m+ufy;
       zDoubleDot = u(3)-Kf*zDot/m+ufz;
       
       fPhi = ((Iy-Iz)/Ix)*thetaDot*sayDot+JTP*thetaDot*wStar/Ix-Kt*L*phiDot/Ix+ufPhi;
       fTheta = ((Iz-Ix)/Iy)*phiDot*sayDot-JTP*phiDot*wStar/Iy-(Kt*L/Iy)*thetaDot+ufTheta;
       fPsi = ((Ix-Iy)/Iz)*phiDot*thetaDot-(Kt*L/Iz)*sayDot+ufSay;
       
       phiDoubleDot = fPhi+u(4)/Ix;
       thetaDoubleDot = fTheta+u(5)/Iy;
       psiDoubleDot = fPsi+u(6)/Iz;
       
         XDOT =  [xDot
                       xDoubleDot
                       yDot
                       yDoubleDot
                       zDot
                       zDoubleDot
                       phiDot
                       phiDoubleDot
                       thetaDot
                       thetaDoubleDot
                       sayDot
                       psiDoubleDot];
 
end
