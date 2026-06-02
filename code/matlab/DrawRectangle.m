function  Pr = DrawRectangle(x,y,theta)
a = x;
b = y;
w = 0.5963;
h = 0.3346;
X = [-w/2 w/2 w/2 -w/2 -w/2];
Y = [h/2 h/2 -h/2 -h/2 h/2];
P = [X;Y];
ct = cos(theta);
st = sin(theta);
R = [ct -st;st ct];
Pr = [R * P];
Pr(1,:) = Pr(1,:)+a;
Pr(2,:) = Pr(2,:)+b;

axis equal;
end