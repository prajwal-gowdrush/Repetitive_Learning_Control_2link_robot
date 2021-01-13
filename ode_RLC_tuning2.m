function Xdot=ode_RLC_tuning2(t,X)
global K Kn Kl gamma alpha beta c1 c2 tvec whathistory tprevious T_earlier_index what T tauhistory

%Assigning states from the input arguments to other variables
q1=X(1); q2=X(2); q1dot=X(3); q2dot=X(4); 
thetahat=X(5:6,1);
q=[q1;q2]; qdot=[q1dot;q2dot]; 

%Defining the desired trajectory
qd1=cos(0.5*t); qd2=2*cos(t);
qd1dot=-0.5*sin(0.5*t); qd2dot=-2*sin(t);
qd1doubledot=-0.25*cos(0.5*t); qd2doubledot=-2*cos(t);
qd=[qd1;qd2]; 
qd_dot=[qd1dot;qd2dot];

%Error signal definitions
e=qd-q; 
edot=qd_dot-qdot;
r= edot + alpha*e;
z=[e; r];

%Regression Matrices
Ys = [sign(q1dot) 0 ; 0 sign(q2dot)];

%%%%%%%%%%%%%%Adaptive Update Law%%%%%%%%%%%%%%%%%%%%%%%%
thetahatdot = gamma*Ys'*r;

if ~isempty(tprevious)
    if (t-tprevious)>10^-20
        record_torque=1;
      if t>T
         what = sat( whathistory(T_earlier_index,:), beta) + (Kl*r)';
      else
         what = (Kl*r)';
      end
      tvec= [tvec;t];
      whathistory = [whathistory ; what];
      while (tvec(end) - tvec(T_earlier_index)) > T
          T_earlier_index = T_earlier_index + 1;
      end
      tprevious = t;
    else
        record_torque=0;
    end
else
    what= (Kl*r)';
    tvec=t;
    whathistory=what;
    tprevious=t;
    record_torque=1;
end

%Term needed in the control law
rho_squared = (c1*norm(z) + c2)^2;


%Control Law
tau= K*r + Kn*rho_squared*r + e + what' + Ys*thetahat;
if abs(tau(2,1))>30
    keyboard;
end
if record_torque == 1
    if ~isempty(tauhistory)
    tauhistory=[tauhistory;tau'];
    else
    tauhistory=tau';
    end
end

%Actual Parameter values (Have not been used in the control law)
p1 = 3.473;
p2 = 0.196;
p3 = 0.242;
fd1 = 5.3;
fd2 = 1.1;
fs1 = 1.2;
fs2 = 0.4;

%Matrices appearing in the dynamics
M= [ p1+2*p3*cos(q2), p2+p3*cos(q2); p2+p3*cos(q2),  p2]; %Inertia Matrix
V= [ -p3*sin(q2)*q2dot, -p3*sin(q2)*(q1dot+q2dot); p3*sin(q2)*q1dot , 0]; %Centrifugal Coriolis Matrix
Fd=[fd1 , 0;  0 , fd2];
Fs=[fs1 0; 0 fs2];

%% Dynamics
Xdot(1,1)= q1dot;
Xdot(2,1)= q2dot;
Xdot(3:4,1)= M\(tau - V*qdot - Fd*qdot - Fs*sign(qdot)); % M\ is inverse of M
Xdot(5:6,1)=thetahatdot;