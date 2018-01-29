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

% FACE
r1 = 86;                                                        % radius of circle to be drawn -> 0 < r < MaxX
xorigin1 = 0;
yorigin1 = 0;
prec1 = 60;                                                     % take 60 points
x1 = linspace(r1 + xorigin1,xorigin1,prec1);
y1 = linspace(yorigin1, r1 + yorigin1, prec1);
x_face = horzcat( x1, -x1, -x1, x1);
y_face = horzcat( y1, y1, -y1, -y1);

% % TRAJECTORY
% Xd = [0 x_face];
% Yd = [0 y_face]; 

Xd = x_face;
Yd = y_face;

% Sample Time for Set-Point Time Vector
SampleTime = TotalTime / (length(Xd)-1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Values you should over-write in Control.m %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% =====================
% Set-Point Time Vector
% =====================
Time       = 0:SampleTime:TotalTime;       % DO NOT CHANGE TotalTime
