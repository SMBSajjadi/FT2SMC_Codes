clc
clear
close all

H=0.001;
T=100;
N=round(T/H);
T=linspace(0,T,N);

u=1.8;%uncertainty
La_h=0.01;
La_v=1;
k=.02;%1/(2*365);%0.7902;
beta=0.1333;%50.333;%
la_v=09;
mioo_h=0.55;
mioo_v=0.0002429;
p=0.15;
sai=0.001;
alpha1=0.288;%0.01;%
alpha2=0.0488;
tao=0.4;%0.01–0.7
b=0.01;%0.1;%
la1=2; la2=1;  la3=01;
phi1=0.5; phi2=.5; phi3 =0.5;
S_h=zeros(1,N);
S_v=zeros(1,N);
E_h=zeros(1,N); Ed=zeros(1,N); Ed_d=zeros(1,N); e1=zeros(1,N); s1=zeros(1,N);
E_v=zeros(1,N);
I_h=zeros(1,N); Id=zeros(1,N); Id_d=zeros(1,N); e2=zeros(1,N); s2=zeros(1,N);
I_v=zeros(1,N); Idv=zeros(1,N); Idv_d=zeros(1,N); e3=zeros(1,N);s3=zeros(1,N);
R_h=zeros(1,N);
u1=zeros(1,N);
u2=zeros(1,N);
u3=zeros(1,N);
Fi1=zeros(1,N);
Fi2=zeros(1,N);
Fi3=zeros(1,N);
E_h(1)=200;
S_h(1)=1100;
I_h(1)=400;
I_v(1)=80;
E_v(1)=700;
S_v(1)=800;
R_h(1)=0;
a2=.055;
a=.055;
ga1=zeros(1,N);ga2=zeros(1,N);ga3=zeros(1,N);
ga1(1)=5; ga2(1)=5;    ga3(1)=5;
teta1=[(1/beta);((alpha1+mioo_h)/beta);(1/beta)];
teta2=[(1/tao);alpha1/tao;b/tao;(sai+mioo_h)/tao;1/tao];
teta3=[1/p;alpha2/p;mioo_v/p;1/p];

% h=1;
% h=i;
for i=1:N-1
    Ed(i)=E_h(1)*exp(-a*T(i));
    Id(i)=I_h(1)*exp(-a*T(i));
    Idv(i)=I_v(1)*exp(-a*T(i));
    Ed_d(i)=-E_h(1)*a*exp(-a*T(i));
    Id_d(i)=-I_h(1)*a*exp(-a*T(i));
    Idv_d(i)=-I_v(1)*a*exp(-a*T(i));
    
    e1(i)=E_h(i)-Ed(i);
    e2(i)=I_h(i)-Id(i);
    e3(i)=I_v(i)-Idv(i);
    if i==1
    s1(i)=H*e1(i);
    s2(i)=H*e2(i);
    s3(i)=H*e3(i);
    else 
        s1(i)=H*e1(i)+s1(i-1);
        s2(i)=H*e2(i)+s2(i-1);
        s3(i)=H*e3(i)+s3(i-1);
    end
    
    Fi1(i)=Ed_d(i)-la1*(e1(i)+5*s1(i));
    Fi2(i)=Id_d(i)-la2*(e2(i)+5*s2(i));
    Fi3(i)=Idv_d(i)-la3*(e3(i)+5*s3(i));
    
    Y1=[-Fi1(i)/S_h(i) -E_h(i)/S_h(i) (ga1(i)*tanh(10*(e1(i)+5*s1(i))))/S_h(i)];
    Y2=[-Fi2(i)/I_h(i) E_h(i)/I_h(i) -1 -1 (ga2(i)*tanh(10*(e2(i)+5*s2(i))))/I_h(i)];
    Y3=[-Fi3(i)/I_v(i) E_v(i)/I_v(i) -1 (ga3(i)*tanh(10*(e3(i)+5*s3(i))))/I_v(i)];
    
    u1(i)=(Y1*teta1*u)+1;
    u2(i)=(Y2*teta2*u);
    u3(i)=(Y3*teta3*u);
    
    ga1_d=phi1*abs(E_h(i)-Ed(i));
    ga2_d=phi2*abs(I_h(i)-Id(i));
    ga3_d=phi3*abs(I_v(i)-Idv(i));
    
    ga1(i+1)=H*ga1_d+ga1(i);
    ga2(i+1)=H*ga2_d+ga2(i);
    ga3(i+1)=H*ga3_d+ga3(i);
    
    B=@(S_h,R_h)La_h+(k*R_h)-((1-u1(i))*(beta*S_h))-(mioo_h*S_h);
    C=@(S_h,E_h)((1-u1(i))*(beta*S_h))-(alpha1+mioo_h)*E_h;
    D=@(E_h,I_h)(alpha1*E_h)-((b+tao*u2(i))*I_h)-(sai+mioo_h)*I_h;
    F=@(R_h,I_h)((b+tao*u2(i))*I_h)-(k+mioo_h)*R_h;
    G=@(S_v)La_v-(1-u1(i))*la_v*S_v-p*u3(i)*S_v-mioo_v*S_v;
    I=@(S_v,E_v)(1-u1(i))*la_v*S_v-p*u3(i)*E_v-(alpha2+mioo_v)*E_v;
    J=@(E_v,I_v)alpha2*E_v-p*u3(i)*I_v-mioo_v*I_v;
    
    K1=B(S_h(i),R_h(i));
    L1=C(S_h(i),E_h(i));
    M1=D(E_h(i),I_h(i));
    N1=F(R_h(i),I_h(i));
    O1=G(S_v(i));
    P1=I(S_v(i),E_v(i));
    Q1=J(E_v(i),I_v(i));
    
    K2=B(S_h(i)+(H/2)*K1,R_h(i)+(H/2)*N1);
    L2=C(S_h(i)+(H/2)*K1,E_h(i)+(H/2)*L1);
    M2=D(E_h(i)+(H/2)*L1,I_h(i)+(H/2)*M1);
    N2=F(R_h(i)+(H/2)*N1,I_h(i)+(H/2)*M1);
    O2=G(S_v(i)+(H/2)*O1);
    P2=I(S_v(i)+(H/2)*O1,E_v(i)+(H/2)*P1);
    Q2=J(E_v(i)+(H/2)*P1,I_v(i)+(H/2)*Q1);
    
    K3=B(S_h(i)+(H/2)*K2,R_h(i)+(H/2)*N2);
    L3=C(S_h(i)+(H/2)*K2,E_h(i)+(H/2)*L2);
    M3=D(E_h(i)+(H/2)*L2,I_h(i)+(H/2)*M2);
    N3=F(R_h(i)+(H/2)*N2,I_h(i)+(H/2)*M2);
    O3=G(S_v(i)+(H/2)*O2);
    P3=I(S_v(i)+(H/2)*O2,E_v(i)+(H/2)*P2);
    Q3=J(E_v(i)+(H/2)*P2,I_v(i)+(H/2)*Q2);
    
    K4=B(S_h(i)+(H)*K3,R_h(i)+(H)*N3);
    L4=C(S_h(i)+(H)*K3,E_h(i)+(H)*L3);
    M4=D(E_h(i)+(H)*L3,I_h(i)+(H)*M3);
    N4=F(R_h(i)+(H)*N3,I_h(i)+(H)*M3);
    O4=G(S_v(i)+(H)*O3);
    P4=I(S_v(i)+(H)*O3,E_v(i)+(H)*P3);
    Q4=J(E_v(i)+(H)*P3,I_v(i)+(H)*Q3);
    
    S_h(i+1)=S_h(i)+(H/6)*(K1+2*K2+2*K3+K4);
    E_h(i+1)=E_h(i)+(H/6)*(L1+2*L2+2*L3+L4);
    I_h(i+1)=I_h(i)+(H/6)*(M1+2*M2+2*M3+M4);
    R_h(i+1)=R_h(i)+(H/6)*(N1+2*N2+2*N3+N4);
    S_v(i+1)=S_v(i)+(H/6)*(O1+2*O2+2*O3+O4);
    E_v(i+1)=E_v(i)+(H/6)*(P1+2*P2+2*P3+P4);
    I_v(i+1)=I_v(i)+(H/6)*(Q1+2*Q2+2*Q3+Q4);
end
E=E_h-Ed;
Ih=I_h-Id;
Iv=I_v-Idv;
% for i=1:N-1
%         Ed(i)=E_h(1)*exp(-a*T(i));
% end
% for i=1:N2-1
%         Ed2(i)=E_h(1)*exp(-a2*T2(i));
% end
% figure(1)
% plot(T,Ed,'linewidth',2)
% hold on
% plot(T2,Ed2,'-.r','linewidth',2)
% xlabel('Time(days)');ylabel('Number of human');
% legend('E_h_d_1:First desired scenario','E_h_d_2:Second desired scenario','latex');
figure(2)
plot(T(1:i),E_h(1:i),'linewidth',2)
hold on
plot(T(1:i),Ed(1:i),'-.r','linewidth',2)
hold on
plot(T(1:i),R_h(1:i),'g','linewidth',2)
xlabel('Time(days)');ylabel('Number of humans');
legend('E_h','E_h_d','R');
figure(3)
plot(T,I_h,'linewidth',2)
hold on
plot(T,Id,'linewidth',2)
legend('I_h','I_h_d')
xlabel('Time(days)');ylabel('Number of humans');
figure(4)
plot(T,I_v,'linewidth',2)
hold on
plot(T,Idv,'linewidth',2)
xlabel('Time(days)');ylabel('Number of mosquitoes')
legend('I_v','I_v_d')
figure(5)
plot(T(1:i-1),u1(1:i-1),'linewidth',2)
hold on
plot(T(1:i-1),u2(1:i-1),'linewidth',2)
hold on
plot(T(1:i-1),u3(1:i-1),'linewidth',2)
xlabel('Time(days)');ylabel('Control inputs')
legend('u_1','u_2','u_3')
figure(6)
plot(T(1:i-1),ga1(1:i-1),'linewidth',2)
hold on
plot(T(1:i-1),ga2(1:i-1),'linewidth',2)
hold on
plot(T(1:i-1),ga3(1:i-1),'linewidth',2)
xlabel('Time(days)');ylabel('Adaptation of robust gains');
legend({'${\widehat{\gamma_1}}$','${\widehat{\gamma_2}}$','${\widehat{\gamma_3}}$'},'Interpreter','latex')
figure(7)
plot(T(1:i),E(1:i),'linewidth',2)
hold on
plot(T(1:i),Ih(1:i),'linewidth',2)
hold on
plot(T(1:i),Iv(1:i),'linewidth',2)
xlabel('Time(days)');ylabel('Tracking errors');
legend('E_h-E_h_d','I_h-I_h_d','I_v-I_v_d')
figure(8)
plot(T(1:i),E_v(1:i),'linewidth',2)
hold on
plot(T(1:i),S_v(1:i),'-.r','linewidth',2)
xlabel('Time(days)');ylabel('Number of mosquitoes');
legend('E_v','S_v');
figure(9)
plot(T(1:99999),u1(1:99999),'linewidth',2)
hold on
plot(T(1:199999),u1(1:199999),'-.r','linewidth',2)
xlabel('Time(days)');ylabel('Use of treated bednets');
legend('u_1 for the first scenario','u_1 for the second scenario');
figure(10)
plot(T(1:99999),u2(1:99999),'linewidth',2)
hold on
plot(T(1:199999),A(1:199999),'-.r','linewidth',2)
xlabel('Time(days)');ylabel('Treatment rate of infected humans');
legend('u_2 for the first scenario','u_2 for the second scenario');
figure(11)
plot(T(1:99999),u3(1:99999),'linewidth',2)
hold on
plot(T(1:199999),u3(1:199999),'-.r','linewidth',2)
xlabel('Time(days)');ylabel('Use of insecticide spray');
legend('u_3 for the first scenario','u_3 for the second scenario');
figure(12)
plot(T(1:99999),I_h(1:99999),'linewidth',2)
hold on
plot(T(1:99999),R_h(1:99999),'linewidth',2)
hold on
plot(T(1:199999),I_h(1:199999),'-.r','linewidth',2)
hold on
plot(T(1:199999),R_h(1:199999),'-.r','linewidth',2)
xlabel('Time(days)');ylabel('Human states');
legend('I_h for the first scenario','R_h for the first scenario','I_h for the second scenario','R_h for the second scenario');