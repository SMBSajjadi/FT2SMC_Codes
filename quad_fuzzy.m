clc;
clear;
close;

T=70;%for0.1-50for1
H=0.001;
N=round(T/H);
T=linspace(0,T,N);

%% Parmeter of system
g=9.8;
% Omega_5=150;
m=2;
d_z=2;
l=0.5;
J_z=8*10^-5;
I_x=48*10^-4;
I_y=48*10^-4;
I_z=81*10^-4;
J_p=2*10^-5;
kt=4*10^-5;
kt_5=2*10^-5;
kd=3*10^-6;
kd_5=1.5*10^-6;
A=I_y-I_z-(4*J_z)-(2*J_p);
B=I_z-I_x+(4*J_z)+(2*J_p);
C=I_x-I_y;


%% Design parmeter of disturbance obserevr
k=1000;
beta=.01;
epsilon=.1;
p0=1;
q0=3;

%% Design parmeter of TSMC
alpha1=5;
beta1=0.1;
p1=5;
q1=7;

Tao=3.7;

Eta=3;

%% Input saturation
umax_z=24.5;%30;
umin_z=-1;%-.00010;

umax_phi=3000;
umin_phi=-3000;

umax_teta=3000;
umin_teta=-3000;

umax_sai=3000;
umin_sai=-3000;

%% Fuzzy
F2=readfis('F8');

%% Output&Input
X1=zeros(1,N);
X2=zeros(1,N);
X3=zeros(1,N);
X4=zeros(1,N);
X5=zeros(1,N);
X6=zeros(1,N);
X7=zeros(1,N);
X8=zeros(1,N);
X9=zeros(1,N);
X10=zeros(1,N);
X11=zeros(1,N);
X12=zeros(1,N);

X1_d=zeros(1,N);
X3_d=zeros(1,N);
X5_d=zeros(1,N);
X7_d=zeros(1,N);
X9_d=zeros(1,N);
X11_d=zeros(1,N);

X1_d_moshtagh=zeros(1,N);
X3_d_moshtagh=zeros(1,N);

X1_d_moshtagh2m=zeros(1,N);
X3_d_moshtagh2m=zeros(1,N);

Omega_1=zeros(1,N);Omega_1_d=zeros(1,N);
Omega_2=zeros(1,N);Omega_2_d=zeros(1,N);
Omega_3=zeros(1,N);Omega_3_d=zeros(1,N);
Omega_4=zeros(1,N);Omega_4_d=zeros(1,N);
Omega_5=zeros(1,N);Omega_5_d=zeros(1,N);Omega_5_p2=zeros(1,N);


Dhat_x=zeros(1,N);
Z_x=zeros(1,N);
Q_x=zeros(1,N);
D_x=zeros(1,N);

Dhat_y=zeros(1,N);
Z_y=zeros(1,N);
Q_y=zeros(1,N);

Dhat_z=zeros(1,N);
Z_z=zeros(1,N);
Q_z=zeros(1,N);

Dhat_phi=zeros(1,N);
Z_phi=zeros(1,N);
Q_phi=zeros(1,N);

Dhat_teta=zeros(1,N);
Z_teta=zeros(1,N);
Q_teta=zeros(1,N);

Dhat_sai=zeros(1,N);
Z_sai=zeros(1,N);
Q_sai=zeros(1,N);


vr_eq_phi=zeros(1,N);
vr_phi=zeros(1,N);
v_eq_phi=zeros(1,N);
v_phi=zeros(1,N);
u_phi=zeros(1,N);

vr_eq_teta=zeros(1,N);
vr_teta=zeros(1,N);
v_eq_teta=zeros(1,N);
v_teta=zeros(1,N);
u_teta=zeros(1,N);

vr_eq_sai=zeros(1,N);
vr_sai=zeros(1,N);
v_eq_sai=zeros(1,N);
v_sai=zeros(1,N);
u_sai=zeros(1,N);

u_eq_x=zeros(1,N);
u_x=zeros(1,N);

u_eq_y=zeros(1,N);
u_y=zeros(1,N);

vr_eq_z=zeros(1,N);
vr_z=zeros(1,N);
v_eq_z=zeros(1,N);
v_z=zeros(1,N);
u_z=zeros(1,N);


S_phi=zeros(1,N);
S_d_phi=zeros(1,N);
S2_phi=zeros(1,N);
S2_d_phi=zeros(1,N);

S_teta=zeros(1,N);
S_d_teta=zeros(1,N);
S2_teta=zeros(1,N);
S2_d_teta=zeros(1,N);

S_sai=zeros(1,N);
S_d_sai=zeros(1,N);
S2_sai=zeros(1,N);
S2_d_sai=zeros(1,N);

S_x=zeros(1,N);
S_d_x=zeros(1,N);
S2_x=zeros(1,N);
S2_d_x=zeros(1,N);

S_y=zeros(1,N);
S_d_y=zeros(1,N);
S2_y=zeros(1,N);
S2_d_y=zeros(1,N);

S_z=zeros(1,N);
S_d_z=zeros(1,N);
S2_z=zeros(1,N);
S2_d_z=zeros(1,N);

delta_u=zeros(1,N);
dhat_z=zeros(1,N);

%% Initial condition
X1(1)=0;
X2(1)=0;
X3(1)=0;
X4(1)=0;
X5(1)=0;
X6(1)=0;
X7(1)=2;%1
X8(1)=0;
X9(1)=2;%1
X10(1)=0;
X11(1)=1;
X12(1)=0;

X1_d(1)=0;
X2_d(1)=0;
X3_d(1)=0;
X4_d(1)=0;
X5_d(1)=0;
X6_d(1)=0;
X7_d(1)=0;
X8_d(1)=0;
X9_d(1)=0;
X10_d(1)=0;
X11_d(1)=0;
X12_d(1)=0;


Z_x(1)=0;
Z_z(1)=0;


% Rectangular Trajectory
%% Z

z_d = @(t) (t<50)*3 + (t>=50)*0;
X11_d_moshtagh = zeros(N,1);
X11_d_moshtagh2m = zeros(N,1);

%% X

x_d=@(t) (t<10)*4 + (t>=10 && t<30)*3 + (t>=30)*4;
X7_d_moshtagh=zeros(N,1);
X7_d_moshtagh2m=zeros(N,1);

%% Y

y_d = @(t) (t<20)*4 + (t>=20 && t<40)*3 + (t>=40)*4;
X9_d_moshtagh=zeros(N,1);
X9_d_moshtagh2m=zeros(N,1);

%% Psi

psi_d= @(t) (t<50)*pi/4 + (t>=50)*0;
X5_d_moshtagh=zeros(N,1);
X5_d_moshtagh2m=zeros(N,1);


%% Fuzzy TSMC
for i=1:N-1
    
%     Omega_5(i+1)=sqrt((m-dhat_z(i))/(2*kt_5));
    Omega_5(i+1)=370;
    Omega_5_p2(i+1)=Omega_5(i+1)^2;
    
    % syms Omega_1_p2 Omega_2_p2 Omega_3_p2 Omega_4_p2 u_z u_phi u_teta u_sai kt kt_5 Omega_5_p2 l kd kd_5
    % OMEGA=solve('-u_z+kt*(Omega_1_p2+Omega_2_p2+Omega_3_p2+Omega_4_p2)+kt_5*Omega_5_p2==0','-u_phi+kt*l*(Omega_2_p2-Omega_4_p2)==0','-u_teta+kt*l*(Omega_3_p2-Omega_1_p2)==0','-u_sai+kd*(Omega_1_p2+Omega_3_p2-Omega_2_p2-Omega_4_p2)+kd_5*Omega_5_p2==0',Omega_1_p2,Omega_2_p2,Omega_3_p2,Omega_4_p2);
    % OMEGA.Omega_1_p2
    Omega_1_p2=-(2*kd*u_teta(i) - kd*l*u_z(i) - kt*l*u_sai(i) + Omega_5_p2(i+1)*kd*kt_5*l + Omega_5_p2(i+1)*kd_5*kt*l)/(4*kd*kt*l);
    Omega_2_p2=(2*kd*u_phi(i) + kd*l*u_z(i) - kt*l*u_sai(i) - Omega_5_p2(i+1)*kd*kt_5*l + Omega_5_p2(i+1)*kd_5*kt*l)/(4*kd*kt*l);
    Omega_3_p2=(2*kd*u_teta(i)+ kd*l*u_z(i) + kt*l*u_sai(i) - Omega_5_p2(i+1)*kd*kt_5*l - Omega_5_p2(i+1)*kd_5*kt*l)/(4*kd*kt*l);
    Omega_4_p2=-(2*kd*u_phi(i) - kd*l*u_z(i) + kt*l*u_sai(i) + Omega_5_p2(i+1)*kd*kt_5*l - Omega_5_p2(i+1)*kd_5*kt*l)/(4*kd*kt*l);
    Omega_1(i+1)=sqrt(Omega_1_p2);
    Omega_2(i+1)=sqrt(Omega_2_p2);
    Omega_3(i+1)=sqrt(Omega_3_p2);
    Omega_4(i+1)=sqrt(Omega_4_p2);
    
    Omega_1_d(i+1)=(Omega_1(i+1)-Omega_1(i));
    
    Omega_2_d(i+1)=(Omega_2(i+1)-Omega_2(i));
    
    Omega_3_d(i+1)=(Omega_3(i+1)-Omega_3(i));
    
    Omega_4_d(i+1)=(Omega_4(i+1)-Omega_4(i));
    
    Omega_5_d(i+1)=(Omega_5(i+1)-Omega_5(i));
    
    
    
    Omega=Omega_1(i+1)+Omega_3(i+1)-Omega_2(i+1)-Omega_4(i+1);
    Omega_d=Omega_1_d(i+1)+Omega_3_d(i+1)-Omega_2_d(i+1)-Omega_4_d(i+1);
    
    
    
    Q_x(i+1)=-k*S_x(i)-beta*sign(real(S_x(i)))-epsilon*(S_x(i)^(p0/q0))+(u_z(i)/m)*u_x(i);
    Z_x(i+1)=(Q_x(i+1)*(H))+Z_x(i);
    S_x(i+1)=Z_x(i+1)-X8(i);
    S_d_x(i+1)=(S_x(i+1)-S_x(i));
    Dhat_x(i+1)=-k*S_x(i+1)-beta*sign(real(S_x(i+1)))-epsilon*(S_x(i+1)^(p0/q0));
    
    
    Q_z(i+1)=-k*S_z(i)-beta*sign(real(S_z(i)))-epsilon*(S_z(i)^(p0/q0))-g*sign(real(S_z(i)))+ vr_z(i);
    Z_z(i+1)=(Q_z(i+1)*(H))+Z_z(i);
    S_z(i+1)=Z_z(i+1)-X12(i);
    S_d_z(i+1)=(S_z(i+1)-S_z(i));
    Dhat_z(i+1)=-k*S_z(i+1)-beta*sign(real(S_z(i+1)))-epsilon*(S_z(i+1)^(p0/q0))-g*sign(real(S_z(i)))+g;
    dhat_z(i+1)=Dhat_z(i+1)-((cos(X1(i))*cos(X3(i)))/m)*(delta_u(i))+(Tao/(((cos(X1(i))*cos(X3(i)))/m)^2+Tao))*vr_z(i);   
    
    
    Q_y(i+1)=-k*S_y(i)-beta*sign(real(S_y(i)))-epsilon*(S_y(i)^(p0/q0))-(u_z(i)/m)*u_y(i);
    Z_y(i+1)=(Q_y(i+1)*(H))+Z_y(i);
    S_y(i+1)=Z_y(i+1)-X10(i);
    S_d_y(i+1)=(S_y(i+1)-S_y(i));
    Dhat_y(i+1)=-k*S_y(i+1)-beta*sign(real(S_y(i+1)))-epsilon*(S_y(i+1)^(p0/q0));
    
    
    Q_phi(i+1)=-k*S_phi(i)-beta*sign(real(S_phi(i)))-epsilon*(S_phi(i)^(p0/q0))-abs((1/I_x)*(A*X4(i)*X6(i)-J_z*X4(i)*Omega-J_p*X4(i)*Omega_5(i+1)))*sign(real(S_phi(i)))+vr_phi(i); %-(abs(f(x)))*o_0(1,1)=0
    Z_phi(i+1)=(Q_phi(i+1)*(H))+Z_phi(i);
    S_phi(i+1)=Z_phi(i+1)-X2(i);
    S_d_phi(i+1)=(S_phi(i+1)-S_phi(i));
    Dhat_phi(i+1)=-k*S_phi(i+1)-beta*sign(real(S_phi(i+1)))-epsilon*(S_phi(i+1)^(p0/q0))-abs((1/I_x)*(A*X4(i)*X6(i)-J_z*X4(i)*Omega-J_p*X4(i)*Omega_5(i+1)))*sign(real(S_phi(i)))-((1/I_x)*(A*X4(i)*X6(i)-J_z*X4(i)*Omega-J_p*X4(i)*Omega_5(i+1)));%-(abs(f(x)))*o_0(1,1)-f(x)=0
    
  
    Q_teta(i+1)=-k*S_teta(i)-beta*sign(real(S_teta(i)))-epsilon*(S_teta(i)^(p0/q0))-abs((1/I_y)*(B*X2(i)*X6(i)+J_z*X2(i)*Omega+J_p*X2(i)*Omega_5(i+1)))*sign(real(S_teta(i)))+vr_teta(i); %-(abs(f(x)))*o_0(1,1)=0
    Z_teta(i+1)=(Q_teta(i+1)*(H))+Z_teta(i);
    S_teta(i+1)=Z_teta(i+1)-X4(i);
    S_d_teta(i+1)=(S_teta(i+1)-S_teta(i));
    Dhat_teta(i+1)=-k*S_teta(i+1)-beta*sign(real(S_teta(i+1)))-epsilon*(S_teta(i+1)^(p0/q0))-abs((1/I_y)*(B*X2(i)*X6(i)+J_z*X2(i)*Omega+J_p*X2(i)*Omega_5(i+1)))*sign(real(S_teta(i)))-((1/I_y)*(B*X2(i)*X6(i)+J_z*X2(i)*Omega+J_p*X2(i)*Omega_5(i+1)));%-(abs(f(x)))*o_0(1,1)-f(x)=0

    
    Q_sai(i+1)=-k*S_sai(i)-beta*sign(real(S_sai(i)))-epsilon*(S_sai(i)^(p0/q0))-abs((1/I_z)*(C*X2(i)*X4(i)-J_z*Omega_d-J_p*Omega_5_d(i+1)))*sign(real(S_sai(i)))+vr_sai(i); %-(abs(f(x)))*o_0(1,1)=0
    Z_sai(i+1)=(Q_sai(i+1)*(H))+Z_sai(i);
    S_sai(i+1)=Z_sai(i+1)-X6(i);
    S_d_sai(i+1)=(S_sai(i+1)-S_sai(i));
    Dhat_sai(i+1)=-k*S_sai(i+1)-beta*sign(real(S_sai(i+1)))-epsilon*(S_sai(i+1)^(p0/q0))-abs((1/I_z)*(C*X2(i)*X4(i)-J_z*Omega_d-J_p*Omega_5_d(i+1)))*sign(real(S_sai(i)))-((1/I_z)*(C*X2(i)*X4(i)-J_z*Omega_d-J_p*Omega_5_d(i+1)));%-(abs(f(x)))*o_0(1,1)-f(x)=0

    
    X11_d(i)=z_d(T(i));
    X7_d(i)=x_d(T(i));
    X9_d(i)=y_d(T(i));
    X5_d(i)=psi_d(T(i));
    
    
    S2_z(i+1)=X12(i)-X11_d_moshtagh(i)+alpha1*(X11(i)-X11_d(i))+beta1*((X11(i)-X11_d(i))^(p1/q1))+S_z(i+1);
    S2_d_z(i+1)=(S2_z(i+1)-S2_z(i));
    o_z=evalfis([abs(S2_z(i+1))*sign(real(S2_z(i+1))) abs(S2_d_z(i+1))*sign(real(S2_d_z(i+1)))],F2);
    vr_eq_z(i+1)=g+X11_d_moshtagh2m(i)-(alpha1*(X12(i)-X11_d_moshtagh(i)))-(beta1*(p1/q1)*((X11(i)-X11_d(i))^((p1-q1)/q1))*(X12(i)-X11_d_moshtagh(i)))-Dhat_z(i+1);
    vr_z(i+1)=vr_eq_z(i+1)-(o_z(1,2)*o_z(1,1));
    v_eq_z(i+1)=(((cos(X1(i))*cos(X3(i)))/m)/(((cos(X1(i))*cos(X3(i)))/m)^2+Tao))*vr_eq_z(i+1);
    v_z(i+1)=(((cos(X1(i))*cos(X3(i)))/m)/(((cos(X1(i))*cos(X3(i)))/m)^2+Tao))*vr_z(i+1);
    vs_max_z=(1/(2*sqrt(Tao)))*o_z(1,2);
    vs_min_z=-vs_max_z;
    u_tilda_max_z=umax_z+vs_min_z;
    u_tilda_min_z=umin_z+vs_max_z;
    if (v_eq_z(i+1)>u_tilda_max_z)
        delta_u(i+1)=umax_z-v_eq_z(i+1);
        u_z(i+1)=umax_z;
    elseif ((u_tilda_min_z<v_eq_z(i+1))&&(v_eq_z(i+1)<u_tilda_max_z))
        delta_u(i+1)=0;
        u_z(i+1)=v_z(i+1);
    elseif (v_eq_z(i+1)<u_tilda_min_z)
         delta_u(i+1)=umin_z-v_eq_z(i+1);
        u_z(i+1)=umin_z;
    end
    
    
    S2_x(i+1)=X8(i)-X7_d_moshtagh(i)+alpha1*(X7(i)-X7_d(i))+beta1*((X7(i)-X7_d(i))^(p1/q1))+S_x(i+1);
    S2_d_x(i+1)=(S2_x(i+1)-S2_x(i));
    o_x=evalfis([abs(S2_x(i+1))*sign(real(S2_x(i+1))) abs(S2_d_x(i+1))*sign(real(S2_d_x(i+1)))],F2);
    u_eq_x(i+1)=X7_d_moshtagh2m(i)-(alpha1*(X8(i)-X7_d_moshtagh(i)))-(beta1*(p1/q1)*((X7(i)-X7_d(i))^((p1-q1)/q1))*(X8(i)-X7_d_moshtagh(i)))-Dhat_x(i+1);
    u_x(i+1)=u_eq_x(i+1)*(m/u_z(i+1))-((o_x(1,2)*o_x(1,1))*m/u_z(i+1));    
    
    X3_d(i+1)=asin(u_x(i+1));
    X3_d_moshtagh(i+1)=(X3_d(i+1)- X3_d(i));
    X3_d_moshtagh2m(i+1)=(X3_d_moshtagh(i+1)- X3_d_moshtagh(i));
    
    
    S2_y(i+1)=X10(i)-X9_d_moshtagh(i)+alpha1*(X9(i)-X9_d(i))+beta1*((X9(i)-X9_d(i))^(p1/q1))+S_y(i+1);
    S2_d_y(i+1)=(S2_y(i+1)-S2_y(i));
    o_y=evalfis([abs(S2_y(i+1))*sign(real(S2_y(i+1))) abs(S2_d_y(i+1))*sign(real(S2_d_y(i+1)))],F2);
    u_eq_y(i+1)=X9_d_moshtagh2m(i)-(alpha1*(X10(i)-X9_d_moshtagh(i)))-(beta1*(p1/q1)*((X9(i)-X9_d(i))^((p1-q1)/q1))*(X10(i)-X9_d_moshtagh(i)))-Dhat_y(i+1);
    u_y(i+1)=-(u_eq_y(i+1)*(m/u_z(i+1)))+((o_y(1,2)*o_y(1,1))*m/u_z(i+1));

    X1_d(i+1)=asin(u_y(i+1)/cos(X3_d(i+1)));
    X1_d_moshtagh(i+1)=(X1_d(i+1)- X1_d(i));
    X1_d_moshtagh2m(i+1)=(X1_d_moshtagh(i+1)- X1_d_moshtagh(i));
    
    
    S2_phi(i+1)=X2(i)-X1_d_moshtagh(i+1)+alpha1*(X1(i)-X1_d(i+1))+beta1*((X1(i)-X1_d(i+1))^(p1/q1))+S_phi(i+1);
    S2_d_phi(i+1)=(S2_phi(i+1)-S2_phi(i));
    o_phi=evalfis([abs(S2_phi(i+1))*sign(real(S2_phi(i+1))) abs(S2_d_phi(i+1))*sign(real(S2_d_phi(i+1)))],F2);
    vr_eq_phi(i+1)=-((A*X4(i)*X6(i)-J_z*X4(i)*Omega-J_p*X4(i)*Omega_5(i+1))/I_x)+X1_d_moshtagh2m(i+1)-(alpha1*(X2(i)-X1_d_moshtagh(i+1)))-(beta1*(p1/q1)*((X1(i)-X1_d(i+1))^((p1-q1)/q1))*(X2(i)-X1_d_moshtagh(i+1)))-Dhat_phi(i+1);
    vr_phi(i+1)=vr_eq_phi(i+1)-(o_phi(1,2)*o_phi(1,1));
    v_eq_phi(i+1)=((1/I_x)/((1/I_x)^2+Tao))*vr_eq_phi(i+1);
    v_phi(i+1)=((1/I_x)/((1/I_x)^2+Tao))*vr_phi(i+1);
    vs_max_phi=((1/I_x)/((1/I_x)^2+Tao))*o_phi(1,2);
    vs_min_phi=-vs_max_phi;
    u_tilda_max_phi=umax_phi+vs_min_phi;
    u_tilda_min_phi=umin_phi+vs_max_phi;
    if (v_eq_phi(i+1)>u_tilda_max_phi)
        u_phi(i+1)=umax_phi;
    elseif ((u_tilda_min_phi<v_eq_phi(i+1))&&(v_eq_phi(i+1)<u_tilda_max_phi))
        u_phi(i+1)=v_phi(i+1);
    elseif (v_eq_phi(i+1)<u_tilda_min_phi)
        u_phi(i+1)=umin_phi;
    end
    
    S2_teta(i+1)=X4(i)-X3_d_moshtagh(i+1)+alpha1*(X3(i)-X3_d(i+1))+beta1*((X3(i)-X3_d(i+1))^(p1/q1))+S_teta(i+1);
    S2_d_teta(i+1)=(S2_teta(i+1)-S2_teta(i));
    o_teta=evalfis([abs(S2_teta(i+1))*sign(real(S2_teta(i+1))) abs(S2_d_teta(i+1))*sign(real(S2_d_teta(i+1)))],F2);
    vr_eq_teta(i+1)=-((B*X2(i)*X6(i)+J_z*X2(i)*Omega+J_p*X2(i)*Omega_5(i+1))/I_y)+X3_d_moshtagh2m(i+1)-(alpha1*(X4(i)-X3_d_moshtagh(i+1)))-(beta1*(p1/q1)*((X3(i)-X3_d(i+1))^((p1-q1)/q1))*(X4(i)-X3_d_moshtagh(i+1)))-Dhat_teta(i+1);
    vr_teta(i+1)=vr_eq_teta(i+1)-(o_teta(1,2)*o_teta(1,1));
    v_eq_teta(i+1)=((1/I_y)/((1/I_y)^2+Tao))*vr_eq_teta(i+1);
    v_teta(i+1)=((1/I_y)/((1/I_y)^2+Tao))*vr_teta(i+1);
    vs_max_teta=((1/I_y)/((1/I_y)^2+Tao))*o_teta(1,2);
    vs_min_teta=-vs_max_teta;
    u_tilda_max_teta=umax_phi+vs_min_teta;
    u_tilda_min_teta=umin_phi+vs_max_teta;
    if (v_eq_teta(i+1)>u_tilda_max_teta)
        u_teta(i+1)=umax_teta;
    elseif ((u_tilda_min_teta<v_eq_teta(i+1))&&(v_eq_teta(i+1)<u_tilda_max_teta))
        u_teta(i+1)=v_teta(i+1);
    elseif (v_eq_teta(i+1)<u_tilda_min_teta)
        u_teta(i+1)=umin_teta;
    end
    
    S2_sai(i+1)=X6(i)-X5_d_moshtagh(i)+alpha1*(X5(i)-X5_d(i))+beta1*((X5(i)-X5_d(i))^(p1/q1))+S_sai(i+1);
    S2_d_sai(i+1)=(S2_sai(i+1)-S2_sai(i));
    o_sai=evalfis([abs(S2_sai(i+1))*sign(real(S2_sai(i+1))) abs(S2_d_sai(i+1))*sign(real(S2_d_sai(i+1)))],F2);
    vr_eq_sai(i+1)=-((C*X2(i)*X4(i)-J_z*Omega_d-J_p*Omega_5_d(i+1))/I_z)+X5_d_moshtagh2m(i)-(alpha1*(X6(i)-X5_d_moshtagh(i)))-(beta1*(p1/q1)*((X5(i)-X5_d(i))^((p1-q1)/q1))*(X6(i)-X5_d_moshtagh(i)))-Dhat_sai(i+1);
    vr_sai(i+1)=vr_eq_sai(i+1)-(o_sai(1,2)*o_sai(1,1));
    v_eq_sai(i+1)=((1/I_z)/((1/I_z)^2+Tao))*vr_eq_sai(i+1);
    v_sai(i+1)=((1/I_z)/((1/I_z)^2+Tao))*vr_sai(i+1);
    vs_max_sai=((1/I_z)/((1/I_z)^2+Tao))*o_sai(1,2);
    vs_min_sai=-vs_max_sai;
    u_tilda_max_sai=umax_phi+vs_min_sai;
    u_tilda_min_sai=umin_phi+vs_max_sai;
    if (v_eq_sai(i+1)>u_tilda_max_sai)
        u_sai(i+1)=umax_sai;
    elseif ((u_tilda_min_sai<v_eq_sai(i+1))&&(v_eq_sai(i+1)<u_tilda_max_sai))
        u_sai(i+1)=v_sai(i+1);
    elseif (v_eq_sai(i+1)<u_tilda_min_sai)
        u_sai(i+1)=umin_sai;
    end
    
    D_x(i)=sin(.8*pi*T(i))+cos(.2*pi*T(i))+sin(.4*pi*T(i))+cos(.6*pi*T(i));

    
    F=@(X2)X2;
    G=@(X4,X6)(1/I_x)*(A*X4*X6-J_z*X4*Omega-J_p*X4*Omega_5(i+1)+ u_phi(i+1));
    Q=@(X4)X4;
    W=@(X2,X6)(1/I_y)*(B*X2*X6+J_z*X2*Omega+J_p*X2*Omega_5(i+1)+u_teta(i+1));
    E=@(X6)X6;
    R=@(X2,X4)(1/I_z)*(C*X2*X4-J_z*Omega_d-J_p*Omega_5_d(i+1)+u_sai(i+1));
    U=@(X8)X8;
    I=@(T)((u_z(i+1)*u_x(i+1))/m)+sin(.8*pi*T)+cos(.2*pi*T)+sin(.4*pi*T)+cos(.6*pi*T);
    O=@(X10)X10;
    P=@(X2)-(u_z(i+1)*u_y(i+1))/m;%X??
    S=@(X12)X12;
    D=@(X1,X3)(1/m)*(u_z(i+1)*cos(X1)*cos(X3)-(m*g))-d_z;
    
    K1=F(X2(i));
    L1=G(X4(i),X6(i));
    A1=Q(X4(i));
    H1=W(X2(i),X6(i));
    J1=E(X6(i));
    Z1=R(X2(i),X4(i));
    C1=U(X8(i));
    V1=I(T(i));
    B1=O(X10(i));
    N1=P(X2(i));%X??
    M1=S(X12(i));
    Q1=D(X1(i),X3(i));
    
    K2=F(X2(i)+((H/2)*L1));
    L2=G(X4(i)+((H/2)*H1),X6(i)+((H/2)*Z1));
    A2=Q(X4(i)+((H/2)*H1));
    H2=W(X2(i)+((H/2)*L1),X6(i)+((H/2)*Z1));
    J2=E(X6(i)+((H/2)*Z1));
    Z2=R(X2(i)+((H/2)*L1),X4(i)+((H/2)*H1));
    C2=U(X8(i)+((H/2)*V1));
    V2=I(T(i));
    B2=O(X10(i)+((H/2)*N1));
    N2=P(X2(i)+((H/2)*L1));
    M2=S(X12(i)+((H/2)*Q1));
    Q2=D(X1(i)+((H/2)*K1),X3(i)+((H/2)*A1));
    
    K3=F(X2(i)+((H/2)*L2));
    L3=G(X4(i)+((H/2)*H2),X6(i)+((H/2)*Z2));
    A3=Q(X4(i)+((H/2)*H2));
    H3=W(X2(i)+((H/2)*L2),X6(i)+((H/2)*Z2));
    J3=E(X6(i)+((H/2)*Z2));
    Z3=R(X2(i)+((H/2)*L2),X4(i)+((H/2)*H2));
    C3=U(X8(i)+((H/2)*V2));
    V3=I(T(i));
    B3=O(X10(i)+((H/2)*N2));
    N3=P(X2(i)+((H/2)*L2));
    M3=S(X12(i)+((H/2)*Q2));
    Q3=D(X1(i)+((H/2)*K2),X3(i)+((H/2)*A2));
    
    K4=F(X2(i)+((H)*L3));
    L4=G(X4(i)+((H)*H3),X6(i)+((H)*Z3));
    A4=Q(X4(i)+((H)*H3));
    H4=W(X2(i)+((H)*L3),X6(i)+((H)*Z3));
    J4=E(X6(i)+((H)*Z3));
    Z4=R(X2(i)+((H)*L3),X4(i)+((H)*H3));
    C4=U(X8(i)+((H)*V3));
    V4=I(T(i));
    B4=O(X10(i)+((H)*N3));
    N4=P(X2(i)+((H)*L3));
    M4=S(X12(i)+((H)*Q3));
    Q4=D(X1(i)+((H)*K3),X3(i)+((H)*A3));
    
    X1(i+1)=X1(i)+(H/6)*(K1+2*K2+2*K3+K4);
    X2(i+1)=X2(i)+(H/6)*(L1+2*L2+2*L3+L4);
    X3(i+1)=X3(i)+(H/6)*(A1+2*A2+2*A3+A4);
    X4(i+1)=X4(i)+(H/6)*(H1+2*H2+2*H3+H4);
    X5(i+1)=X5(i)+(H/6)*(J1+2*J2+2*J3+J4);
    X6(i+1)=X6(i)+(H/6)*(Z1+2*Z2+2*Z3+Z4);
    X7(i+1)=X7(i)+(H/6)*(C1+2*C2+2*C3+C4);
    X8(i+1)=X8(i)+(H/6)*(V1+2*V2+2*V3+V4);
    X9(i+1)=X9(i)+(H/6)*(B1+2*B2+2*B3+B4);
    X10(i+1)=X10(i)+(H/6)*(N1+2*N2+2*N3+N4);
    X11(i+1)=X11(i)+(H/6)*(M1+2*M2+2*M3+M4);
    X12(i+1)=X12(i)+(H/6)*(Q1+2*Q2+2*Q3+Q4);
    
end


X11_d(N)=3;
X7_d(N)=5*sin(T(N));
X9_d(N)=5*cos(T(N));
X5_d(N)=pi/4;

D_x(N)=sin(.8*pi*T(N))+cos(.2*pi*T(N))+sin(.4*pi*T(N))+cos(.6*pi*T(N));


d_z1=zeros(1,N);
for i=1:N
    d_z1(i)=-2;
end

u_z1=zeros(1,N);
for i=1:N
    u_z1(i)=umax_z;
end

%% Plot
figure(1)
subplot(221);
plot(T,X7_d,'-.r','linewidth',2)
hold on
plot(T, X7,'b','linewidth',2)
xlabel('Time(s)');ylabel('X(m)')
legend('Desired trajectory','Simulated trajectory');
subplot(222);
plot(T,X9_d,'-.r','linewidth',2)
hold on
plot(T, X9,'b','linewidth',2)
xlabel('Time(s)');ylabel('Y(m)')
legend('Desired trajectory','Simulated trajectory');
subplot(223);
plot(T,X11_d,'-.r','linewidth',2)
hold on
plot(T, X11,'b','linewidth',2)
xlabel('Time(s)');ylabel('Z(m)')
legend('Desired trajectory','Simulated trajectory');
subplot(224);
plot(T,X5_d,'-.r','linewidth',2)
hold on
plot(T,X5,'b','linewidth',2)
xlabel('Time(s)');ylabel('\psi(rad)')
legend('Desired trajectory','Simulated trajectory');


figure(2)
subplot(221);
plot(T, u_phi,'b')
xlabel('Time(s)');
ylabel('u_{\phi} (N.m)')
legend('u_{\phi}')
grid on
xlim([0 70])

subplot(222);
plot(T, u_teta,'b')
xlabel('Time(s)')
ylabel('u_{\theta} (N.m)')
legend('u_{\theta}')
grid on
xlim([0 70])

subplot(223);
plot(T, u_sai,'b')
xlabel('Time(s)')
ylabel('u_{\psi} (N.m)')
legend('u_{\psi}')
grid on
xlim([0 70])

subplot(224);
plot(T, u_z1,'-.r','linewidth',2)
hold on
plot(T, u_z,'b')
xlabel('Time(s)')
ylabel('u_{z} (N.m)')
legend('u_{z}')
grid on
xlim([0 70])



figure(3)
plot3(X7_d,X9_d,X11_d,'-.r','linewidth',2)
hold on
plot3(X7,X9,X11,'b','linewidth',2)
xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)')
legend('Desired trajectory','Simulated trajectory');

% figure(4)
% plot(T, Omega_5,'b','linewidth',2)
% xlabel('Time(s)');ylabel('\Omega_5(rad/s)')



figure(5)
xlabel('Time(s)');ylabel('D_x(N)')
legend({'${D_x}$','${\hat D_x}$'},'Interpreter','latex');

figure(6)
plot(T, d_z1,'-.r','linewidth',2)
hold on
plot(T, dhat_z,'b','linewidth',2)
plot(T,D_x,'-.r','linewidth',2)
hold on
plot(T, Dhat_x,'b','linewidth',2)
xlabel('Time(s)');ylabel('D_z(N)')
legend({'${D_z}$','${\hat D_z}$'},'Interpreter','latex');
