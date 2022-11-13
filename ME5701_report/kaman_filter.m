%%  Kalman Filter
clc,clear;

% X(K)=F*X(K-1)+Q
% Y(K)=H*X(K)+R

% 
t=0.1:0.01:1;
L=length(t);

% real signal x, and observation y
x=zeros(1,L);
y=x;
y2=x;

x(1)=0.01;
for i=2:L
    x(i)=t(i)^2;
%     x(i)=x(i-1)*1;
    % add noise
    y(i)=x(i)+normrnd(0,0.1);
    y2(i)=x(i)+normrnd(0,0.01);
end
plot(t,x,t,y,'LineWidth',2);
%%
x3=zeros(1,L);
y3=zeros(1,L);
x3(1)=0.1;
y3(1)=0.1^2;
for i=2:L
    % real signal
    x3(i)=sin(3*x3(i-1));
    y3(i)=x3(i)^2+normrnd(0,0.1);
end
plot(t,x3,t,y3,'LineWidth',2);

%%
% filter algorithm

% model 1
% X(K)=X(K-1)+Q
% Y(K)=X(K)+R
% Q~N(0,1), R~N(0,1)
F1=1;
H1=1;
Q1=0.1;
R1=1;
Xplus1=zeros(1,L);
% initial value Xplus1(1)~N(0.01,0.01^2)
Xplus1(1)=0.01;
Pplus1=0.01^2;
% X(K)minus=F*X(K-1)plus
% P(K)minus=F*P(K-1)plus*F'+Q
% K=P(K)minus*H'*inv(H*P(K)minus*H'+R)
% X(K)plus=X(K)minus+K*(y(k)-H*X(K)minus)
% P(K)plus=(I-K*H)*P(K)minus
for i=2:L
    % prediction
    Xminus1=F1*Xplus1(i-1);
    Pminus1=F1*Pplus1*F1'+Q1;
    % update
    K1=(Pminus1*H1')/(H1*Pminus1*H1'+R1);
    Xplus1(i)=Xminus1+K1*(y(i)-H1*Xminus1);
    Pplus1=(eye(1)-K1*H1)*Pminus1;
end
plot(t,x,'r',t,y,'g',t,Xplus1,'b','LineWidth',2);


%%
% model 2
% X(K)=X(K-1)+X'(K-1)*dt+X"(K-1)*dt^2*(1/2!)+Q2
% Y(K)=X(K)+R, R~N(0,1)

% observation equation
% X=[X(K); X'(K); X"(K)]
% Y(K)=H*X+R, H=[1 0 0]

% prediction equation
% X(K)=X(K-1)+X'(K-1)*dt+X"(K-1)*dt^2*(1/2!)+Q2
% X'(K)=0*X(K-1)+X'(K-1)+X"(K-1)*dt+Q3
% X"(K)=0*X(K-1)+0*X'(K-1)+X"(K-1)+Q4
% F= 1    dt    0.5*dt^2
%    0    1      dt
%    0    0      1
% Q= Q2 0  0
%     0 Q3 0
%     0 0  Q4

dt=t(2)-t(1);
F2=[1,dt,0.5*dt^2;
    0,1,dt;
    0,0,1];
H2=[1,0,0];
Q2=[1,0,0;0,0.01,0;0,0,0.0001];
R2=20;
% initial value
Xplus2=zeros(3,L);
Xplus2(1,1)=0.1^2;
Xplus2(2,1)=0;
Xplus2(3,1)=0;
Pplus2=[0.01,0,0;  0,0.01,0;   0,0,0.0001];
for i=2:L
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

%%
% for x3 with model 2
dt=t(2)-t(1);
F2=[1,dt,0.5*dt^2;
    0,1,dt;
    0,0,1];
H2=[1,0,0];
Q2=[1,0,0;0,0.01,0;0,0,0.0001];
R2=20;
% initial value
Xplus2=zeros(3,L);
Xplus2(1,1)=0.1^2;
Xplus2(2,1)=0;
Xplus2(3,1)=0;
Pplus2=[0.01,0,0;  0,0.01,0;   0,0,0.0001];
for i=2:L
    Xminus2=F2*Xplus2(:,i-1);
    Pminus2=F2*Pplus2*F2'+Q2;
    K2=Pminus2*H2'/(H2*Pminus2*H2'+R2);
    Xplus2(:,i)=Xminus2+K2*(y3(i)-H2*Xminus2);
    Pplus2=(eye(3)-K2*H2)*Pminus2;
end
plot(t,x3,'r',t,y3,'g',t,Xplus2(1,:),'b','LineWidth',2);

%%
% sensor fusion
% Y1(K)=X(K)+R1
% Y2(K)=X(K)+R2
% H=[1;1] for model 1
% H=[1 0 0; 1 0 0] for model 2

F3=[1,dt,0.5*dt^2;
    0,1,dt;
    0,0,1];
H3=[1,0,0;  1 0 0];
Q3=[1,0,0;0,0.01,0;0,0,0.0001];
R3=[3,0; 0,5];

% initial value
Xplus3=zeros(3,L);
Xplus3(1,1)=0.1^2;
Xplus3(2,1)=0;
Xplus3(3,1)=0;
Pplus3=[0.01,0,0;  0,0.01,0;   0,0,0.0001];
for i=2:L
    Xminus3=F3*Xplus3(:,i-1);
    Pminus3=F3*Pplus2*F3'+Q3;
    K3=Pminus3*H3'/(H3*Pminus3*H3'+R3);
    Y=zeros(2,1);
    Y(1,1)=y(i);
    Y(2,1)=y2(i);
    Xplus3(:,i)=Xminus3+K3*(Y-H3*Xminus3);
    Pplus3=(eye(3)-K3*H3)*Pminus3;
end
plot(t,x,'r',t,y,'g',t,Xplus2(1,:),'b',t,Xplus3(1,:),'k','LineWidth',2);
xlabel('Time/s');
ylabel('Amplitude');
legend('original','observed','estimated','sensor fusion');



















