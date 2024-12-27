function [u,e] = SlidingModeControl(x,XD,xDoubleDotd,K,...
                                                      fPhi,fTheta,fPsi)
   
    m = 1;
    Kf = 1e-6;
    e = x - XD;

    Ix = 0.0081;        % X Axis Moment of Intertia
    Iy = Ix;               % Y Axis Moment of Intertia
    Iz = 0.0142;        % Z Axis MOMENT of Intertia

    S = [K(1)*e(1)+e(2)
           K(2)*e(3)+e(4)
           K(3)*e(5)+e(6)
           K(4)*e(7)+e(8)
           K(5)*e(9)+e(10)
           K(6)*e(11)+e(12)];

    %% Altitude Control

    ux = -K(1)*e(2)+xDoubleDotd(1)+Kf*x(2)/m-K(7)*tanh(S(1));
    uy = -K(2)*e(4)+xDoubleDotd(2)+Kf*x(4)/m-K(8)*tanh(S(2));
    uz = -K(3)*e(6)+xDoubleDotd(3)+Kf*x(6)/m-K(9)*tanh(S(3));

%     max_u = 3;
%     ux = max(min(ux, max_u), -max_u);
%     uy = max(min(uy, max_u), -max_u);
%     uz = max(min(uz, max_u), -max_u);

    %% Atitude Control

    uPhi = (-Ix)*(fPhi+K(4)*e(8)-xDoubleDotd(4)+K(10)*tanh(S(4)));
    uTheta =  (-Iy)*(fTheta+K(5)*e(10)-xDoubleDotd(5)+K(11)*tanh(S(5)));
    uPsi = (-Iz)*(K(6)*e(12)-xDoubleDotd(6)+fPsi+K(12)*tanh(S(6)));

    u = [ux,uy,uz,uPhi,uTheta,uPsi]';

end