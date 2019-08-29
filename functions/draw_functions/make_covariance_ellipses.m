function p= make_covariance_ellipses(x,P)
% compute ellipses for plotting state covariances
N= 10;
inc= 2*pi/N;
phi= 0:inc:2*pi;

lenx= length(x);
lenf= (lenx-3)/2;
p= zeros (2,(lenf+1)*(N+2));

ii=1:N+2;
p(:,ii)= make_ellipse(x(1:2), P(1:2,1:2), 2, phi);

ctr= N+3;
for i=1:lenf
    ii= ctr:(ctr+N+1);
    jj= 2+2*i; jj= jj:jj+1;
    
    p(:,ii)= make_ellipse(x(jj), P(jj,jj), 2, phi);
    ctr= ctr+N+2;
end

