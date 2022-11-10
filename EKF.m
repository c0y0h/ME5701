% code for EKF
clc,clear;
t=0.01:0.01:1;
n=length(t);
x=zeros(1,n);
y=zeros(1,n);
x(1)=0.1;
y(1)=0.1^2;
for i=2:n
    % real signal
    x(i)=sin(3*x(i-1));
    y(i)=x(i)^2+normrnd(0,0.1);
end

plot(t,x,'r',t,y,'b','LineWidth',2);
%% EKF
Xplus=zeros(1,n);
% initial value
Pplus=0.1;
Xplus(1)=0.1;
Q=0.0001;
R=1;
for i=2:n
    % predict
    A=3*cos(3*Xplus(i-1));
    Xminus=sin(3*Xplus(i-1));
    Pminus=A*Pplus*A'+Q;
    % update
    C=2*Xminus;
    K=Pminus*C'/(C*Pminus*C'+R);
    Xplus(i)=Xminus+K*(y(i)-Xminus^2);
    Pplus=(eye(1)-K*C)*Pminus;
end

plot(t,x,'r',t,y,'g',t,Xplus,'b','LineWidth',2);
xlabel('Time/s');
ylabel('Amplitude');
legend('original','observed','estimated');

%% still use KF
dt=t(2)-t(1);
F2=[1,dt,0.5*dt^2;
    0,1,dt;
    0,0,1];
H2=[1,0,0];
Q2=[1,0,0;0,0.01,0;0,0,0.0001];
R2=20;
% initial value
Xplus2=zeros(3,n);
Xplus2(1,1)=0.1^2;
Xplus2(2,1)=0;
Xplus2(3,1)=0;
Pplus2=[0.01,0,0;  0,0.01,0;   0,0,0.0001];
for i=2:n
    Xminus2=F2*Xplus2(:,i-1);
    Pminus2=F2*Pplus2*F2'+Q2;
    K2=Pminus2*H2'/(H2*Pminus2*H2'+R2);
    Xplus2(:,i)=Xminus2+K2*(y(i)-H2*Xminus2);
    Pplus2=(eye(3)-K2*H2)*Pminus2;
end
plot(t,x,'r',t,y,'g',t,Xplus2(1,:),'b','LineWidth',2);
xlabel('Time/s');
ylabel('Amplitude');
legend('original','observed','estimated');



















