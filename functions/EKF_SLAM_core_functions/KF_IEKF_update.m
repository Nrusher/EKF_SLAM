function [x,P] = KF_IEKF_update(x,P, z,R, hfun,hjac, N)
%function [x,P] = KF_IEKF_update(x,P, z,R, hfun,hjac, N)
%
% INPUTS:
%   x - x(k|k-1) - predicted state
%   P - P(k|k-1) - predicted covariance
%   z - observation
%   R - observation uncertainty
%   hfun - function for computing the innovation, given the non-linear observation model: v = hfun(x,z);
%   hjac - function for computing the observation model jacobian: H = hjac(x);
%   N - number of iterations of the IEKF update
%
% OUTPUTS:
%   x - x(k|k) - a posteri state
%   P - P(k|k) - a posteri covariance
%
% Uses iterated EKF (cite Bar-Shalom 01 - p406). This implementation is rather inefficient for 
% SLAM, as it involves the inversion of P (ie, the update is O(n^3) for n landmarks.
%
% Tim Bailey 2004

xo= x; % prior values
Po= P;
Poi= inv(P);
Ri= inv(R);

for i=1:N % iterate solution
    H= feval(hjac,x);
    P= calculate_P(Po,H,R);
    
    v= feval(hfun,x,z); % to cope with discontinuous models, need this form rather than: v= z - feval(hfun,x); 
    x= calculate_x(v,x,P,xo,Poi,H,Ri);    
end

H= feval(hjac,x); % final iteration 
P= calculate_P(Po,H,R);

%
%
%

function P= calculate_P(P,H,R)
HP= H*P;
PHt= P*H';
S= H*PHt + R;
P= P - PHt * inv(S) * HP;
P= make_symmetric(P); % for assurance

function x= calculate_x(v,x,P,xo,Poi,H,Ri)
M1= P * H' * Ri; 
M2= P * Poi * (x-xo);
x= x + M1*v - M2;

function P= make_symmetric(P)
P= (P+P')/2;
