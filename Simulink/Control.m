% This script sets the controller parameters for the SLS 3-D Printer

% ================
% CONTROLLER GAINS
% ================

% Enter optimized PID values here.
% No more than 3 significant figures per gain value.

% MECHANISMS USED FOR TUNING

% Gain Tuners
% SF0=1.051;
% SF1=1;

% % PID0 Tuners
% SFP0 = 0.73;
% SFI0 = 1.25;
% SFD0 = 1.30;

% % PID1 Tuners
% SFP1 = 1.4;
% SFI1 = 1;
% SFD1 = 1.1;

% % STARTING GAIN VALUES
% K0U = SF0*0.0717;
% K1U = SF1*0.00183;

% % PID0
% P0 = SFP0*(1.95)*(K0U);
% I0 = SFI0*(96.77)*(K0U);
% D0 = SFD0*(K0U)

% % PID1
% P1 = SFP1*(49.17)*(K1U);
% I1 = (0)*(K1U)*SFI1;
% D1 = SFD1*(K1U);


% K0U = SF0*89.28596;
% K1U = SF1*5.85;

% PID0 = [SFP0*0.1398 SFI0*6.9384 SFD0*0.0717];
% PID1 = [SFP1*0.0900 SFI1*0 SFD1*0.0018];

% PID0 Final Values
P0 = 1.07e-1;
I0 = 9.12;
D0 = 9.820e-2;

% PID1 Final Values
P1 = 1.260e-1;
I1 = 0;
D1 = 2.000e-3;

PID0 = [P0 I0 D0];
PID1 = [P1 I1 D1];

FB0=Sens0^(-1); 
FB1=Sens1^(-1);


% =====================
% Set-Point Time Vector
% =====================

% TIME VECTOR WAS NOT MODIFIED

% The Time Vector is stored in a variable called "Time".
% It's initial value is equally spaced for all samples and is
% set in TRAJECTORY.M
%
% Redefine this vector here to optimize the build time of the part.
% You can define it analytically or type in the elements manually but
% you must not change the length because it must contain one value for
% each Xd/Yd position pair.
% In the Matlab window, enter "length(Time)" to see how big it is.

% The Time vector must range from 0 to TotalTime

%Time       = 0:SampleTime:TotalTime;       % DO NOT CHANGE TotalTime
