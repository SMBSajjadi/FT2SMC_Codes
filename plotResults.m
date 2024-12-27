function plotResults(t,x,Xd,u)
    %% A. Plot the States
    
    f1 = figure(1);
    Name = {'x','y','z'};
    NameReference = {'x_{d}','y_{d}','z_{d}'};
                 
    for i=1:3
        
        subplot(3,1,i)
        plot(t,(x(2*i-1,:)),'r','LineWidth',2)
        hold on
        plot(t,(Xd(2*i-1,:)),'k--','LineWidth',1.45)
        grid on
        xlabel('Time (s)','InterPreter','Latex')
        legend(Name{i},NameReference{i},'FontWeight','bold')

    end
    
    f8 = figure(8);
    Name = {'\phi','\theta','\psi'};
    NameReference = {'\phi_{d}','\theta_{d}','\psi_{d}'};
    ii = 0;
    
    for i=3:5
        
        ii = ii + 1;
        subplot(3,1,ii)
        plot(t,180/pi*(x(2*i+1,:)),'r','LineWidth',2)
        hold on
        plot(t,180/pi*((Xd(2*i+1,:))),'k--','LineWidth',1.45)
        grid on
        xlabel('Time (s)','InterPreter','Latex')
        legend(Name{ii},NameReference{ii})
        ylabel([Name{ii}, num2str(' (Degree)')])
        
    end
%     
%     %% Control Signals Plot
%     
    f3 = figure(3);
    U_Name = {'F_{T}','\tau_{\phi}','\tau_{\theta}','\tau_{\psi}'};
    uMainZ = sqrt(u(1,:).^2+u(2,:).^2+(9.81+u(3,:)).^2);
    uMain = [uMainZ
                  u(4,:)
                  u(5,:)
                  u(6,:)];
    Ulabel = {'Total Thrust (N)','Roll Torque (N.m)','Pitch Torque (N.m)','Yaw Torque (N.m)'};

    for i=1:4
    
        subplot(2,2,i)
        plot(t,uMain(i,:),'m','LineWidth',1.5)
        xlabel('Time(s)','InterPreter','Latex')
        grid on
        legend(U_Name{i},'FontWeight','bold')
        xlim([0 50])
        ylabel(Ulabel{i})

    end
%     
%     f9 = figure(9);
%     ColorU = {'r','b',[0.5 0.1 0.3]};
%     U_Name = {'\tau_{\phi}','\tau_{\theta}','\tau_{\psi}'};
%     
%     subIter = 0;
%     
%     for i=size(u,1)/2+1:n
%     
%         subIter = subIter + 1;
%         subplot(3,1,subIter)
%         plot(t,u(i,:),'LineWidth',2,'Color',ColorU{subIter})
%         xlabel('Time(s)','InterPreter','Latex')
%         grid on
%         legend(U_Name{subIter})
%         
%     end
   
   
    %% Desired 3D Trajectory
    
    f5 = figure(5);
    X = x(1,:);
    y = x(3,:);
    z = x(5,:);
    
    plot3(X,y,z,'r--','LineWidth',2.25)
    xlabel('x','FontWeight','Bold')
    ylabel('y','FontWeight','Bold')
    zlabel('z','FontWeight','Bold')
    hold on
    plot3(Xd(1,:),Xd(3,:),Xd(5,:),'Color','b','LineWidth',1.5)
    title('3D Trajectory of the Quadrotor')
    grid on
    legend('X','X_{desired}')
    
    %% Tracking Error
    
    f6 = figure(6);
    Name_e = {'e_{x}','e_{y}','e_{z}',...
                      'e_{\phi}','e_{\theta}','e_{\psi}'};
    for i=1:6
        
        subplot(3,2,i)
        RandomColor = unifrnd(0,1,1,3);
        plot(t,x(2*i-1,:)-Xd(2*i-1,:),'LineWidth',2,'Color',RandomColor);
        xlabel('Time (s)','FontWeight','Bold')
        legend(Name_e{i})
        grid on
        
    end

    %% Move Figures
    
    movegui(f1,'center')
    movegui(f5,'south')
    movegui(f6,'east')
    movegui(f8,'west')
    movegui(f3,'southeast')

end