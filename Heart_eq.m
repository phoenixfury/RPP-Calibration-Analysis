t = linspace(0,2*pi,30);

x = 16 * (sin(t)).^3 * 0.1;

y = 0.1*(13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t));

plot(x,y)