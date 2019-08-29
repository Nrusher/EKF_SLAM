function [x,P]= KF_cholesky_update(x,P,v,R,H)
%function [x,P]= KF_cholesky_update(x,P,v,R,H)
%
% Calculate the KF (or EKF) update given the prior state [x,P]
% the innovation [v,R] and the (linearised) observation model H.
% The result is calculated using Cholesky factorisation, which
% is more numerically stable than a naive implementation.
%
% Tim Bailey 2003
% Developed by Jose Guivant 

PHt= P*H';
S= H*PHt + R;

S= (S+S')*0.5; % make symmetric
SChol= chol(S);

SCholInv= inv(SChol); % triangular matrix
W1= PHt * SCholInv;
W= W1 * SCholInv';

x= x + W*v; % update 
P= P - W1*W1';
