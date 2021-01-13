%% MAINSCRIPT - REPETITIVE LEARNING CONTROL
clc; clear; close all;

global K Kn Kl gamma alpha beta c1 c2 whathistory tprevious T_earlier_index tvec T tauhistory

%%%%%%%%%%%%%%%%%%%Tunable Controller Parameters%%%%%%%%%%%%%%%%%%%%%%%%%%%
K=diag([30;6]);     %Constant multiplying 'r' in the control law
Kn=diag([2;0.05]);    %Constant multiplying 'rho^2 r' in the control law
Kl=diag([2;1]);    %Constant multiplying 'r' in w_hat
beta=6;   %The maximum absolute value that the saturation function can take (less than 4.12 for our problem)
alpha=1;  %Constant in the definition of the error signal 'r'
gamma=diag([300 ; 300]); %Learning rate matrix in the adaptive update law

%%%%%%%%% Coefficients of square of Polynomial-Upperbound(rho)to be determined by trail and error %%%%%%%%%%%%%
c1=1; c2=1;

T=4*pi; %Time-period of the repetitive desired trajectory

%%%%%%%%Initializing %%%%%%%%%%%%%%%
whathistory=[];
tauhistory=[];
tprevious=[];
T_earlier_index=1;

%% Augmented Initial State:
% Initial State:
q10=3; q20=4.5; q1dot0=0; q2dot0=0;
x0=[q10;q20;q1dot0;q2dot0];

% Initial Values of the adaptive estimates: 
%(We pick these based on our expectations of the actual parameter values)
fs1hat=2; fs2hat=1.5;
thetahat0=[fs1hat;fs2hat];

%Augmented Initial State
X0=[x0;thetahat0];

%% Simulation time
t_sim=50;

%% Obtaining the solution to the ode based on the control law specified in ode_RLC
tspan=[0 t_sim];
options = odeset('AbsTol',1e-3,'RelTol',1e-3);
[time,X_Sol]=ode113(@ode_RLC_tuning2,tspan,X0,options);

%% Extracting the trajectories of the states and the estimates from the ODE solution matrix X_Sol:
q1=X_Sol(:,1);
q2=X_Sol(:,2);
q1dot=X_Sol(:,3);
q2dot=X_Sol(:,4);
fs1hat=X_Sol(:,5);
fs2hat=X_Sol(:,6);
%% Computing desired trajectories, errors and controls from the ode solution for the plots:

%Desired trajectories:
qd1=cos(0.5*time); qd2=2*cos(time);
qd1dot=-0.5*sin(0.5*time); qd2dot=-2*sin(time);
qd1doubledot=-0.25*cos(0.5*time); qd2doubledot=-2*cos(time);

%Actual Parameter values (Have not been used in the control law)
fs1 = 1.2;
fs2 = 0.4;

%% Errors
e1=qd1-q1; e2=qd2-q2;
e1dot=qd1dot-q1dot; e2dot=qd2dot-q2dot;
fs1error= fs1-fs1hat; fs2error= fs2-fs2hat;
r1=e1dot+alpha*e1;
r2=e2dot+alpha*e2;

%% Controls
tau1= tauhistory(:,1);
tau2= tauhistory(:,2);

%% Plots
% figure;
subplot(2,2,1);
plot(time,q1,'r',time,q2,'b',time,qd1,'r--',time,qd2,'b--','LineWidth',1.5);
xlabel('Time');
ylabel('States');
title('Desired Vs Actual Trajectories');
xlim([0 time(end)]);
grid on;
ax = gca;
ax.GridLineStyle = ':';
ax.GridAlpha = 0.3;
ax.FontSize = 16;
ax.LineWidth = 1.4;
leg1 = legend('$q_1$','$q_2$','$q_{d1}$','$q_{d2}$');
set(leg1,'Interpreter','latex');
hold on;

% figure;
subplot(2,2,2);
plot(time,e1,'r',time,e2,'b','LineWidth',1.5);
xlabel('Time');
ylabel('Tracking Errors');
title('Tracking Errors Vs Time');
xlim([0 time(end)]);
grid on;
ax = gca;
ax.GridLineStyle = ':';
ax.GridAlpha = 0.3;
ax.FontSize = 16;
ax.LineWidth = 1.4;
leg3 = legend('$e_1$','$e_2$');
set(leg3,'Interpreter','latex');
hold on;

% figure;
subplot(2,2,3);
plot(time,fs1hat,'r',time,fs2hat,'b','LineWidth',1.5);
xlabel('Time');
ylabel('Adaptive Estimates');
title('Adaptive Estimates Vs Time');
xlim([0 time(end)]);
grid on;
ax = gca;
ax.GridLineStyle = ':';
ax.GridAlpha = 0.3;
ax.FontSize = 16;
ax.LineWidth = 1.4;
leg2 = legend('$\widehat{f}_{s_{1}}$','$\widehat{f}_{s_{2}}$');
set(leg2,'Interpreter','latex');
hold on;

% figure;
subplot(2,2,4);
plot(time,fs1error,'r',time,fs2error,'b','LineWidth',1.5);
xlabel('Time');
ylabel('Actual value - Adaptive Estimate');
title('Parameter Estimate Errors');
xlim([0 time(end)]);
grid on;
ax = gca;
ax.GridLineStyle = ':';
ax.GridAlpha = 0.3;
ax.FontSize = 16;
ax.LineWidth = 1.4;
leg2 = legend('$f_{s_{1}} - \widehat{f}_{s_{1}}$' , '$ f_{s_{2}} - \widehat{f}_{s_{2}}$');
set(leg2,'Interpreter','latex');
hold on;

figure;
plot(tvec,whathistory(:,1),'b',tvec,whathistory(:,2),'r','LineWidth',1.5);
xlabel('Time');
ylabel('Repetitive Learning Term');
title('Repetitive Learning Term Vs Time');
xlim([0 time(end)]);
grid on;
ax = gca;
ax.GridLineStyle = ':';
ax.GridAlpha = 0.3;
ax.FontSize = 16;
ax.LineWidth = 1.4;
leg4 = legend('$\widehat{w}_1$','$\widehat{w}_2$');
set(leg4,'Interpreter','latex');
hold on;

figure;
plot(tvec,tau1,'r',tvec,tau2,'b','LineWidth',1.5);
xlabel('Time');
ylabel('Controls');
title('Controls Vs Time');
xlim([0 time(end)]);
grid on;
ax = gca;
ax.GridLineStyle = ':';
ax.GridAlpha = 0.3;
ax.FontSize = 16;
ax.LineWidth = 1.4;
leg5 = legend('$\tau_1$','$\tau_2$');
set(leg5,'Interpreter','latex');
hold on;

%% The following lines are useful for tuning the gains:
maxlink1torque=max(abs(tau1))
maxlink2torque=max(abs(tau2))