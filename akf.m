function [dk,vk]=akf(d,a,Td,Ta,q,r)

% Jieming Niu 08/2013
%
% Perform Adaptive Kalman filter on accelerometer and GPS data in terms of 
% variance component estimation to obtain broadband displacements and
% velocities(see Bock et al., 2011). The time series have to be aligned on
% the FIRST sample and Td/Ta must be an integer.
%
% The code is modified from D. Melgar's function 'kalmand'
% (https://github.com/dmelgarm/Kalman/blob/master/kalmand.m)
%
% Inputs
% d         displacement time series
% a         acceleration time series
% Td        displacement sample rate in seconds
% Ta        acceleration sample rate in seconds
% q         a prior process noise variance (Accel.)
% r         a prior measurement noise variance (GPS)
%
% 
% Outputs
% dk        adaptive kalman filtered displacement
% vk        adaptive kalman filtered velocity


% Structural preeliminaries
if size(d,1)<size(d,2)  %turn to column vector
    d=d';
end
if size(a,1)<size(a,2)  %ditto
    a=a';
end
la=size(a,1);
% Initalize output vectors
dk=zeros(la,1);
vk=zeros(la,1);
% Initialize state variable matrices 
x=[0;0];
% Initialize state covariance matrix
P=[1 0;0 1];
% Initialize State-Space matrices
A=[1 Ta;0 1];
B=[Ta^2/2;Ta];
H=[1 0];
% Initalize noise covariance matrices, which are regarded as the inverses
% of co-weight matrix
r=r.^2;
Q=[q*Ta^3/3 q*Ta^2/2;q*Ta^2/2 q*Ta];
R=r/Td;
% Acceleration to dispalcement sampling ratio
ratio=rnd(Td/Ta);

% ADAPTIVE FILTER
% k counts acceleration data, i counts gps data
i=1;
sizegps=size(d,1);
% If need to output the estimate of process noise variance 
% q=zeros(sizegps,1);
for k=1:la
    % if GPS is available
    if i < sizegps && isintdiv(k-1,ratio)==1
        Qf=Q;
        % Calculate weight matrix
        PL=0.01/R(i);
        % Predict state
        xp=A*x+B*a(k);
        % Construct pseudo residual equation
        v=H*xp-d(i);
        
        % Estimate process noise variance
        while 1
            Qp=A*P*A'+Qf;
            sig_predict=(v'*PL*v-size(R(i),1))/(PL*H*Qp*H');
            sig_pro=(v'*PL*v-sig_predict*trace(PL*H*A*Qp*A'*H')...
                -size(R(i),1))/(PL*H*Qf*H');
            % Iteration ends
            if sig_pro < 1.00001 && sig_pro > 0.99999
                %compute Kalman gain
                K=Qp*H'/(H*Qp*H'+R(i));
                %update state
                x=xp+K*(d(i)-H*xp);
                % update covariance
                P=(eye(2)-K*H)*Qp*(eye(2)-K*H)'+K*R(i)*K';
                break;
            else
                Qf=sig_pro*Qf;
            end
        end
        % Output the estimate of process noise variance
        % q(i)=sqrt(sqrt(det(Qf)*12/Ta^4)/100);
        i=i+1;
    else
        x=A*x+B*a(k);
        P=A*P*A'+Q;
    end
    % Update output variables
    dk(k)=x(1);
    vk(k)=x(2);
end
dk=dk';
vk=vk';



function rnum=rnd(num)

%D. Melgar 10/2010
%Round down or up to the nearest integer

rnum=num-floor(num);
if rnum<0.5
    rnum=floor(num);
else
    rnum=ceil(num);
end

function f=isintdiv(a,b)

%D. Melgar 06/2010
%Decide if a/b produces an integer
%Return 1 if true 0 if false

if a/b - floor(a/b) > 0
    f=0;
else
    f=1;
end
