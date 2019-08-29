function [zf,idf,zn, table]= data_associate_known(x,z,idz, table)
%function [zf,idf,zn, table]= data_associate_known(x,z,idz, table)
%
% For simulations with known data-associations, this function maintains
% a feature/observation lookup table. It returns the updated table, the
% set of associated observations and the set of observations to new features.

% x 当前的位姿
% table - 用来记录某个路标是否被观测到

zf= []; 
zn= [];
idf= []; 
idn= [];

% find associations (zf) and new features (zn)
for i=1:length(idz)
    ii= idz(i);
    % 检测之前是否观测到该路标
    if table(ii) == 0 % new feature
        zn= [zn z(:,i)];
        idn= [idn ii];
    else
        zf= [zf z(:,i)];
        idf= [idf table(ii)];
    end
end

% add new feature IDs to lookup table
Nxv= 3; % number of vehicle pose states
Nf= (length(x) - Nxv)/2; % number of features already in map
table(idn)= Nf + (1:size(zn,2)); % add new feature positions to lookup table
