% This script sets the trajectory

%%%%%%%%%%%%%%%%%%%%%%
% Desired Trajectory %
%%%%%%%%%%%%%%%%%%%%%%
% % xy coordinates of circle moving left to right
% num  = 20;                                 % number of cirle segments
% r    = 100;
% x    = ceil(-num/2:.5:num/2-.5);
% x    = x / max(x) * r;                     % scale to radius size
% y    = sqrt(r^2 - x.^2);                   % pythag theorem
% y    = y .* (mod(1:length(y),2) * 2 - 1);  % negate every second element
% z    = ones(size(y));                      % part height

% % Robot XY Trajectory for cross-hatch pattern
% Xd = [0 x y x y];
% Yd = [0 y x y x];
% Zd = [0 z*0 z*5 z*10 z*15];

% % FACE
% r1 = 86;                                                        % radius of circle to be drawn -> 0 < r < MaxX
% xorigin1 = 0;
% yorigin1 = 0;
% prec1 = 60;                                                     % take 60 points
% x1 = linspace(r1 + xorigin1, xorigin1, prec1);
% y1 = sqrt(r1^2 - (x1 - xorigin1).^2) + yorigin1;
% x_face = horzcat( x1, wrev(-x1), -x1, wrev(x1) );
% y_face = horzcat( y1, wrev(y1), -y1, wrev(-y1) );

% % % TRAJECTORY
% % Xd = [0 x_face];
% % Yd = [0 y_face]; 



% % LEFT EYE
% r2 = 14.1421356237;                                                        % radius of circle to be drawn -> 0 < r < MaxX
% xorigin2 = -40;
% yorigin2 = 30;
% prec2 = 60;                                                             % take 60 points
% x2 = linspace(r2, 0, prec2);
% y2 = sqrt(r2^2 - (x2).^2);
% x_lefteye = horzcat( x2  + xorigin2, wrev(-x2) + xorigin2, -x2 + xorigin2, wrev(x2) + xorigin2 );
% y_lefteye = horzcat( y2 + yorigin2, wrev(y2)+ yorigin2, -y2 + yorigin2, wrev(-y2) + yorigin2 );

% % % RIGHT EYE
% r3 = 14.1421356237;                                                        % radius of circle to be drawn -> 0 < r < MaxX
% xorigin3 = 40;
% yorigin3 = 30;
% prec2 = 60;                                                             % take 60 points
% x3 = linspace(r3, 0, prec2);
% y3 = sqrt(r3^2 - (x3).^2);
% x_righteye = horzcat( x3  + xorigin3, wrev(-x3) + xorigin3, -x3 + xorigin3, wrev(x3) + xorigin3 );
% y_righteye = horzcat( y3 + yorigin3, wrev(y3)+ yorigin3, -y3 + yorigin3, wrev(-y3) + yorigin3 );

% % SMILE
% r4 = 60;
% xorigin4 = 0;
% yorigin4 = 0;
% r_smile = r4 * ones(1,16);

% angle_deg = -45:-6:-135;

% angle_rad = angle_deg * pi/180;

% x_smile = r_smile.*cos(angle_rad) + xorigin4;
% y_smile = r_smile.*sin(angle_rad) + yorigin4;

% % ROBOT TRAJECTORY

% % Xd = [x_face x_lefteye x_righteye];
% % Yd = [y_face x_lefteye x_righteye];

% Xd = [x_face x_righteye x_lefteye  x_smile];
% Yd = [y_face y_righteye y_lefteye  y_smile];

% Yd = [ 130 -130 130 -130 100 -100 100 -100 70 -70 70 -70 40 -40 40 -40 10 -10 10 -10];
x = -pi:0.1:pi;
y = 50*sin(x);

% sin(x)), grid on
% Yd = [0 70 -70 70 -70 70 -70 70 -70 70 -70];
% Xd = [0 0 0 0 0 0 0 0 0 0 0];

Yd = [y];
Xd = [x];

% Yd = [25 -25];
% Xd = [0 0];

% Xd = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];

% Sample Time for Set-Point Time Vector
SampleTime = TotalTime / (length(Xd)-1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Values you should over-write in Control.m %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% =====================
% Set-Point Time Vector
% =====================
Time       = 0:SampleTime:TotalTime;       % DO NOT CHANGE TotalTime
