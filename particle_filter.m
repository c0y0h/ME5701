clc,clear;

t=0.01:0.01:1;
x=zeros(1,100);
y=zeros(1,100);

% initial value
x(1)=0.1;
y(1)=0.01^2;

% real data and observed data
for i=2:100
%     x(i)=sin(x(i-1))+5*x(i-1)/(x(i-1)^2+1);
    x(i)=sin(3*x(i-1));
    y(i)=x(i)^2+normrnd(0,0.1);
%     y(i)=x(i)^2;
end

plot(t,x,'r',t,y,'g','LineWidth',2);

%% PF filter
n=100;
xold=zeros(1,n);
xnew=zeros(1,n);
xplus=zeros(1,100);
w=zeros(1,n);
% x0(i) all 0.1,w(i)=1/n
for i=1:n
    xold(i)=0.1;
    w(i)=1/n;
end

for i=2:100
    % prediction
    for j=1:n
%         xold(j)=sin(xold(j))+5*xold(j)/(xold(j)^2+1)+normrnd(0,0.1);  %Q
        xold(i)=sin(3*xold(i-1))+normrnd(0,0.01);
    end

    % update
    for j=1:n
        % w(j)=w(j)*fR(...)
        % fR=(2*pi*R)^(0.5)*exp(-((y(i)-xold(j)^2)^2/(2*R)))
        w(j)=exp(-((y(i)-xold(j)^2)^2/(2*0.1)));
    end

    % normalization
    w=w/sum(w);

    % resample
    % N>1/sum(wi^2)
    c=zeros(1,n);
    c(1)=w(1);
    for j=2:n
        c(j)=c(j-1)+w(j);
    end
    for j=1:n
        a=unifrnd(0,1);
        for k=1:n
            if a<c(k)
                xnew(j)=xold(k);
                break;
            end
        end
    end
    
    % weights are all 1/n after resampling
    for j=1:n
        w(j)=1/n;
    end

    xplus(i)=sum(xnew)/n;
end

plot(t,x,'r',t,xplus,'b','LineWidth',2);




















