l =  0.1785;
Jr = 2.20321*10^-3;
b = 1.27*10^-7;
d = 3.19*10^-11;
g = 9.80665;
m = 0.38;

a1 = 0.8254478845552427;
a2 = -0.00984322923647411;
a3 = 0.15465303926842378;
a4 = 0.004558302644101462;
a5 = -0.8691472972067786;
b1 = 79.74802305321;
b2 = 36.93052509620557;
b3 = 59.78297273762476;

motor1 = 185;
motor2 = 131;
motor3 = 255;
motor4 = 248;
motorR = 0;


pitch = 0.5;
pitch_dot = 0;
roll = 2.3;
roll_dot = 0;
yaw = 3.3;
yaw_dot = 0;
z = 0;
z_dot = 0;
x = 0;
x_dot = 0;
y = 0;
y_dot = 0;

ux = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
uy = cos(roll)*sin(pitch)*cos(yaw) - sin(roll)*sin(yaw);


a24 = yaw*a1 + a2*motorR;
a42 = yaw*a3 + a4*motorR;
a64 = roll_dot*a5;
b81 = cos(roll)*cos(pitch)/m;
b101 = ux/m;
b121 = uy/m;


A = [0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 0 a24 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 a42 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     0 0 0 a64 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 0 0 0 0 0];
 
 B = [0 0 0 0;
      0 b1 0 0;
      0 0 0 0;
      0 0 b2 0;
      0 0 0 0;
      0 0 0 b3;
      0 0 0 0;
      b81 0 0 0;
      0 0 0 0;
      b101 0 0 0;
      0 0 0 0;
      b121 0 0 0];
  
 C = [1 0 0 0 0 0 0 0 0 0 0 0;
      0 1 0 0 0 0 0 0 0 0 0 0;
      0 0 1 0 0 0 0 0 0 0 0 0;
      0 0 0 1 0 0 0 0 0 0 0 0;
      0 0 0 0 1 0 0 0 0 0 0 0;
      0 0 0 0 0 1 0 0 0 0 0 0;
      0 0 0 0 0 0 1 0 0 0 0 0;
      0 0 0 0 0 0 0 1 0 0 0 0;
      0 0 0 0 0 0 0 0 1 0 0 0;
      0 0 0 0 0 0 0 0 0 1 0 0;
      0 0 0 0 0 0 0 0 0 0 1 0;
      0 0 0 0 0 0 0 0 0 0 0 1];
  
  D = [0 0 0 0;
       0 0 0 0;
       0 0 0 0;
       0 0 0 0;
       0 0 0 0;
       0 0 0 0;
       0 0 0 0;
       0 0 0 0;
       0 0 0 0;
       0 0 0 0;
       0 0 0 0;
       0 0 0 0];
 