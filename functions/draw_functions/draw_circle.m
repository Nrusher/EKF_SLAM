function draw_circle(xc,yc,r)
radius = r;
theta=0:pi/100:2*pi;
x = radius*cos(theta) + xc;
y = radius*sin(theta) + yc;
plot(x,y)
end

