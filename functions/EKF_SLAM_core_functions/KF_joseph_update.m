function [x,P]= KF_joseph_update(x,P,v,R,H)
%function [x,P]= KF_joseph_update(x,P,v,R,H)
%
% This module is identical to KF_simple_update() except that
% it uses the Joseph-form covariance update, as shown in 
% Bar-Shalom "Estimation with Applications...", 2001, p302.
%
% Tim Bailey 2003
 
PHt= P*H';
S= H*PHt + R;
Si= inv(S);
Si= make_symmetric(Si);
PSD_check= chol(Si);
W= PHt*Si;
 
x= x + W*v; 
 
% Joseph-form covariance update
C= eye(size(P)) - W*H;
P= C*P*C' + W*R*W';
 
P= P + eye(size(P))*eps; % a little numerical safety
PSD_check= chol(P);
 
function P= make_symmetric(P)
P= (P+P')*0.5;

