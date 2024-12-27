clc;clear;close all;

m = 1;
g = 9.81;

%% Initialization

Ts = 0.001;   %% Must be 0.0001, at least, to be Fully Robust.
tMax = 70;
t0 = 0;
t = t0:Ts:tMax;
N = numel(t);
n = 6;  % Number of DOF (X)

x0 = [2 0 2 0 1 0 0 0 0 0 0 0];       %% Ics
x = zeros(2*n,N);                          %% Compelete State Vector
x(:,1) = x0;

s = @(x) sin(x);
c = @(x) cos(x);

%% Reference Signals

CASE = 1;           %% Linear Trajectory
% CASE = 2;           %% Circular Trajectory
% CASE = 3;         %% Helical Traj

[XD, XDoubleDotD] = setDesiredTrajectory(t,CASE,n);
sayd = XD(11,:);

%% Error Dynamics

e = zeros(2*n,N);
eDot = e;

%% Control Signal Initialization

nU = 6;             %% iN cOnjunction with the Virtual Signals
u = ones(nU,N);
u(3,1) = 0.2*m*g;
W = 100*ones(4,N);

%% SMC Parameters

a = [10 8 6 1 1 5];
K = [3 2 2 1 1 1.5];
kOptimal = [a'
                  K'];

%% Main Loop

tic;

for i=2:N  

    [K1RK,wStar,W(:,i),fPhi,fTheta,fPsi] =...
     Rotor2_Dynamic(t(i-1),x(:,i-1),u(:,i-1));

    x(:,i) = stateCalculation(K1RK,x(:,i-1),u(:,i-1),Ts,t(i-1));
    
    %% Virtual Control Design
    
    ux = u(1,i-1);
    uy = u(2,i-1);
    uz = u(3,i-1);
    
    thetad = atan((ux*c(sayd(i))+uy*s(sayd(i)))/(g+uz));
    phid = atan(c(thetad)*((ux*s(sayd(i))-uy*c(sayd(i)))/(g+uz)));

    %% Desired Roll/Pittch Angles
    
    XD(7,i) = phid;
    XD(9,i) = thetad;

      %% SMC 

      [u(:,i),e(:,i)] = SlidingModeControl(x(:,i),XD(:,i),XDoubleDotD(:,i),kOptimal,...
                                                                              fPhi,fTheta,fPsi);

end
    

%% PlotResults
 
plotResults(t,x,XD,u)
% PlotEstimationResultsOfFaults(t,MainFault,FaultHat)
% PlotFaultAnglesVersusEstimatedOnes(t,FAULT_ANGLES,EstimatedFaultAngles)
