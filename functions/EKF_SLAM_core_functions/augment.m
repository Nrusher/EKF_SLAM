function [x,P]= augment(x,P,z,R)
%function [x,P]= augment(x,P,z,R)
%
% Inputs:
%   x, P - SLAM state and covariance
%   z, R - range-bearing measurements and covariances, each of a new feature
%
% Outputs:
%   x, P - augmented state and covariance
%
% Notes: 
%   - We assume the number of vehicle pose states is three.
%   - Only one value for R is used, as all measurements are assumed to 
%   have same noise properties.
%
% Tim Bailey 2004.

% add new features to state
for i=1:size(z,2)
    [x,P]= add_one_z(x,P,z(:,i),R);
end

%
%

function [x,P]= add_one_z(x,P,z,R)

len= length(x);
r= z(1); b= z(2);
s= sin(x(3)+b); 
c= cos(x(3)+b);

% augment x
x= [x;
    x(1) + r*c;
    x(2) + r*s];

% jacobians
Gv= [1 0 -r*s;
     0 1  r*c];
Gz= [c -r*s;
     s  r*c];
     
% augment P
rng= len+1:len+2;
P(rng,rng)= Gv*P(1:3,1:3)*Gv' + Gz*R*Gz'; % feature cov
P(rng,1:3)= Gv*P(1:3,1:3); % vehicle to feature xcorr
P(1:3,rng)= P(rng,1:3)';
if len>3
    rnm= 4:len;
    P(rng,rnm)= Gv*P(1:3,rnm); % map to feature xcorr
    P(rnm,rng)= P(rng,rnm)';
end
