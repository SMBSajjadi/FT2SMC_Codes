function PlotComparisonResults()
    
    clc;clear;close all;

    t = 0:0.001:70;
    
    xTSMC = load('xTSMC.mat');
    xSMC = load('xSMC.mat');
    xDOBBSC = load('xDOBBSC.mat');
    xDOBBSC = xDOBBSC.x;

    xTSMC = xTSMC.X;
    xTSMC(:,end+1) = xTSMC(:,end);
    xSMC = xSMC.xSMC;

    %% Z

    z_d = @(t) (t<50)*3 + (t>=50)*0;
    zd = z_d(t);
    
    %% X
    
    x_d=@(t) (t<10)*4 + (t>=10 && t<30)*3 + (t>=30)*4;
    xd = zeros(size(t));

    for i=1:numel(t)
            
        xd(i) = x_d(t(i));

    end
    
    %% Y
    
    y_d = @(t) (t<20)*4 + (t>=20 && t<40)*3 + (t>=40)*4;
    yd = zeros(size(t));
    
    for i=1:numel(t)
            
        yd(i) = y_d(t(i));

    end

    %% Psi
    
    psi_d= @(t) (t<50)*pi/4 + (t>=50)*0;
    psid = psi_d(t);

    %% Plot

    f1 = figure(1);
    
    %% 1

    subplot(2,2,1)
    plot(t,xTSMC(1,:),'b','LineWidth',1.1)
    hold on
    grid on
    plot(t,xSMC(1,:),'m','LineWidth',1.1)
    hold on
    grid on
    plot(t,xDOBBSC(end-1,:),'g','LineWidth',1.1)
    hold on
    plot(t,xd,'r--','LineWidth',1.1)
    xlabel('Time (s)','InterPreter','Latex')
    legend('Proposed','ASMC','DOBBSC','x_{d}')
    ylabel('x(m)','fONTweight','bold')
    xlim([0 70])
    ylim([1 5])

    %% 2

    subplot(2,2,2)
    plot(t,xTSMC(2,:),'b','LineWidth',1.1)
    hold on
    grid on
    plot(t,xSMC(2,:),'m','LineWidth',1.1)
    hold on
    plot(t,xDOBBSC(end-3,:),'g','LineWidth',1.1)
    hold on
    plot(t,yd,'r--','LineWidth',1.1)
    xlabel('Time (s)','InterPreter','Latex')
    legend('Proposed','ASMC','DOBBSC','y_{d}')
    ylabel('y(m)','fONTweight','bold')
    xlim([0 70])
    ylim([1 5])

    %% 3

    subplot(2,2,3)
    plot(t,xTSMC(3,:),'b','LineWidth',1.1)
    hold on
    grid on
    plot(t,xSMC(3,:),'m','LineWidth',1.1)
    hold on
    plot(t,xDOBBSC(end-5,:),'g','LineWidth',1.1)
    hold on
    plot(t,zd,'r--','LineWidth',1.1)
    xlabel('Time (s)','InterPreter','Latex')
    legend('Proposed','ASMC','DOBBSC','z_{d}')
    ylabel('z(m)','fONTweight','bold')
    xlim([0 70])
    ylim([-1 4])

    %% 4

    subplot(2,2,4)
    plot(t,xTSMC(4,:),'b','LineWidth',1.1)
    hold on
    grid on
    plot(t,xSMC(4,:),'m','LineWidth',1.1)
    hold on
    plot(t,xDOBBSC(5,:),'g','LineWidth',1.1)
    hold on
    plot(t,psid,'r--','LineWidth',1.1)
    xlabel('Time (s)','InterPreter','Latex')
    legend('Proposed','ASMC','DOBBSC','\psi_{d}')
    ylabel('\psi(rad)','fONTweight','bold')
    xlim([0 70])
    ylim([-1 1])

    %% 3D Trajectory Comparison

    f2 = figure(2);

    plot3(xTSMC(1,:),xTSMC(2,:),xTSMC(3,:),'b','LineWidth',1.1)
    hold on
    grid on
    plot3(xSMC(1,:),xSMC(2,:),xSMC(3,:),'m','LineWidth',1.1)
    hold on
    plot3(xDOBBSC(end-1,:),xDOBBSC(end-3,:),xDOBBSC(end-5,:),'g','LineWidth',1.1)
    hold on
    plot3(xd,yd,zd,'r--','LineWidth',1.1)

    xlabel('x(m)','InterPreter','Latex')
    ylabel('y(m)','InterPreter','LATEX')
    zlabel('z(m)','InterPreter','lATex')

    legend('Proposed ','ASMC ','DOBBSC ','Target Trajectory')

    movegui(f1,'east')
    movegui(f2,'west')

end