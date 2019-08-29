function draw_car(xtrue,r,color)
xc = xtrue(1);
yc = xtrue(2);
phi = xtrue(3);
theta=0:pi/100:2*pi;
x = r*cos(theta) + xc;
y = r*sin(theta) + yc;

x_dir = r*cos(phi)+xc;
y_dir = r*sin(phi)+yc;

plot(x,y,color,'linewidth',2);
hold on;
plot([xc,x_dir],[yc,y_dir],color,'linewidth',2);
hold on;
plot(xc,yc,'.','markersize',26);
end

