% This script contains all physical constants and specifications for the
% SLS 3-D Printer

% Start with a clean slate
% Erase entire Matlab environment
clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Configuration Constants %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
screenDistance = 200;        % distance away from screen -> mm
screenMaxX = 50;            % maxmimum positive x coordinate -> mm
screenMaxY = 50;            % maximum positive y coordinate -> mm

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Q1 Shaft          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
shaftMassQ1 = 0;          % mass of Q1 shaft -> kg
shaftRadiusQ1 = 0.0;       % radius of Q1 shaft -> m
shaftLengthQ1 = 0.0;       % length of Q1 shaft -> m

%%%%%%%%%%%%%%%%%%%%%%
% Physical Constants %
%%%%%%%%%%%%%%%%%%%%%%
G = 9.81;               % m/s^2
FtPerM = 3.28;          % ft / m
RadPerDeg = pi/180;     % rad / deg
DegPerRad = 180/pi;     % deg / rad
RadPSecPerRPM = pi/30;  % (rad/s) / rpm

Big = 1e6;              % General purpose large number
Sml = 1e-6;             % General purpose small number


%%%%%%%%%%%%%%%%%%
% Specifications %
%%%%%%%%%%%%%%%%%%

% Simulation Execution Time
TotalTime = 20;          % (s)

% Amplifier
R1  = 1;               % (Mohm)
R2  = 1;               % (ohm)
C   = 1;                % (uF)
L   = 1;               % (mH)

% Sensor
SensV   = 5;             % Min/Max sensor outpout voltage     (V)
SensAng = 180;           % Min/Max sensor angle               (deg)

% Geometry
Height   = screenDistance;          % Height above build plate           (mm)
LinkR1   = 16;           % Inner radius of wrist frame        (mm)
LinkR2   = 22;           % Outer radius of wrist frame        (mm)
LinkD    = 5;            % Depth of wrist frame               (mm)
LinkOff  = 9;            % Distance from motor face to centre (mm)

% Material & Spring Constant
RhoAl   = 2.7;           % Density of 6061 Al (g/cm^3)
SpringK = 7;             % (mNm/rev)

% Coeff of static friction
% Amount of torque required to set a body in motion which has a given
% amount of radial force being applied to it (usually due to gravity)
uSF    = 700;            % (um) = (uNm/N)

% Joint Limit
JntLim0 = atan(screenMaxX/screenDistance);             % Q0 joint limit (deg)
% JntLim1 = atan(screenMaxY/screenDistance);             % Q1 joint limit (deg)
JntLim1 = 90;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Indices into MotorParam Vector for Maxon motors %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Values at Nominal Voltage
NomV          = 1;
NoLoadSpd     = 2;
NoLoadCurr    = 3;
NomSpd        = 4;
NomTorque     = 5;
NomCurr       = 6;
StallTorque   = 7;
StallCurr     = 8;
MaxEff        = 9;

% Characteristics
TermR         = 10;
TermL         = 11;
TorqueK       = 12;
SpdK          = 13;
SpdTorqueGrad = 14;
MechTimeK     = 15;
RotJ          = 16;

% Thermal Data
ThermRhous    = 17;
ThermRwind    = 18;
ThermTCwind   = 19;
ThermTCmot    = 20;
AmbTemp       = 21;
MaxTemp       = 22;

% Mechanical Data
MaxSpdBall    = 23;
AxialPlayBall = 24;
RadPlayBall   = 25;
MaxAxLdBall   = 26;
MaxFBall      = 27;
MaxRadLdBall  = 28;

% Other Specifications
NoPolePair    = 29;
NoCommSeg     = 30;
Weight        = 31;

% Physical Dimensions
OuterDiam     = 32;
Length        = 33;
