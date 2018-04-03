
% KILL  close all open figures and simulink models.
% Close all figures.
    % h=findall(0);
    % delete(h(2:end));
% delete(findall(0,'Type','figure'));
% % Close all Simulink models.
% bdclose('all');

%============================================%
% 				Choose Motors                %
%============================================%

% Motor Unit Conversions
% ----------------------
 X_MOTOR;                % Default Maxon motor
 Q0 = MotorParam;
 DCMOTOR;
 Q1 = MotorParam;

%============================================%
% 				Motor Parameters             %
%============================================%

% Static Friction Constant
staticFricConst = uSF*10^(-6);     % Static friction coefficient, um --> m

% Q(n) refers to element at index=n of MotorParam. 
% Parameters found in the AMAX22_5W_SB motor file, and the corresponding indices to the parameters are found in CONSTANTS.m

% Q0
I0Nom   = Q0(NomCurr);             % Max average current
I0Stall = Q0(StallCurr);           % Max peak current
Q0Nom   = Q0(NomV);                % Nominal voltage 

% Q1
I1Nom   = Q1(NomCurr);      	   % Max average current
I1Stall = Q1(StallCurr);           % Max peak current
Q1Nom   = Q1(NomV);                % Nominal voltage 

% %============================================%
% % 			 Amplifier Dynamics              %
% %============================================%
% % Compute transfer function of amplifier. 
% % R1, L, and C values are defined in CONSTANTS.m
% AmpR1 		= R1*10^6;		      % Mohm --> ohm
% AmpC 		= C/10^6;			  % uF --> F
% AmpL 		= L/10^3;			  % mH --> H	

% % The amplifier dynamics will be the same for both the motors since they use the same amplifier circuitry

% % Q0
% Amp0n   = [AmpC*AmpR1*R2-AmpL];                     % Numerator: C*R1*R2 - L
% Amp0d   = [AmpL*AmpC*AmpR1 AmpC*AmpR1*R2]           % Denominator: (L*C*R1)s + (C*R1*R2)
% AmpSat0 = Q0Nom;					                % Amplifier saturation set such that the maximum motor voltage is not exceeded (Nominal Voltage) 
% % Q1
% Amp1n   = [AmpC*AmpR1*R2-AmpL];                     % Numerator: C*R1*R2 - L
% Amp1d   = [AmpL*AmpC*AmpR1 AmpC*AmpR1*R2]           % Denominator: (L*C*R1)s + (C*R1*R2)
% AmpSat1 = Q1Nom;					                % Amplifier saturation set such that the maximum motor voltage is not exceeded (Nominal Voltage)

%============================================%
%          Motor Driver Dynamics             %
%============================================%


% The amplifier dynamics will be the same for both the motors since they use the same amplifier circuitry

% Volts
Q1MaxVoltage = 12;
Q0MaxVoltage = 10;

MDSat1 = 10.8;
InputSat1 = 12;

MDSat0 = 10.8;
InputSat1 = 12;

MD1_Linear_Gain = 0.8920*Q1MaxVoltage/255;
MD0_Linear_Gain = 0.8920*Q0MaxVoltage/255;




%============================================%
% 			 System Parameters               %
%============================================%
% Some values that will be used in the calculations of the mechanical dynamics

% Mass of system (motors and ring)
% % --------------------------------------------
% % The values for the motors are given, so just the mass of the ring needs to be calculated.
% % Mass = volume*density
% % NEED TO CHANGE -> TODO
% rIn 	     = LinkR1/10^3;   			        % Inner radius of wrist frame, mm --> m
% rOut         = LinkR2/10^3;   			        % Outer radius of wrist frame, mm --> m
% wristDepth   = LinkD/10^3;	     		        % Depth of wrist frame, mm --> m
% distCentre   = LinkOff/10^3;                    % Distance from motor face to centre, mm --> m
% lengthQ0     = Q0(Length)/10^3;                 % Length of motor Q0, mm --> m
% lengthQ1     = Q1(Length)/10^3;                 % Length of motor Q1, mm --> m
% ringVol      = pi*(rOut^2-rIn^2)*wristDepth;    % Volume of the ring
% ringDensity  = RhoAl*10^3;				        % Density of 6061 Al, g/cm^3 --> kg/m^3: g/cm^3*(1kg/1000g)*(100^3cm^3/1m^3)=kg/m^3*10^3      
% mRing 	     = ringVol*ringDensity;             % Mass of ring = volume * density
% mQ0          = Q0(Weight)/10^3;                 % Mass of motor Q0
% mQ1          = Q1(Weight)/10^3;                 % Mass of motor Q1
% mTotal0       = 2*mQ1 + mRing;                  % Total mass of loads supported by motor Q0
%                                                 % Q0 counterweight not included because it is supported by the bar
%                                                 % Q1 counterweight included because it is supported by motor Q0 and it is assumed ot have the same mass as the motor

% mTotal1 = shaftMassQ1;                          % Q1 total mass

xholderMass = 1/1000; %kg
xholderDepth = 7/1000; 
xholderWidth = 18.26/1000;
xholderHeight = 10/1000;

laserRadius = 3.5*10^(-3);
laserHeight = 21*10^(-3);
laserMass = 2*10^(-3);

% --------------------------------------------

% Encoder Q1
encoderQ1Mass = 5.6*10^(-3);                    % mass in kg: experimentally obtained
encoderQ1Radius = 24*10^(-3);                   % m: from datasheet
encoderQ1Height = 1.63*10^(-3);                 %m: measured using calipers

% Encoder Q0
encoderQ0Mass = 5.6*10^(-3);                    % mass in kg: experimentally obtained
encoderQ0Radius = 24*10^(-3);                   % m: from datasheet
encoderQ0Height = 1.63*10^(-3);                 %m: measured using calipers

% Motor X


%============================================%
% 		Q0 - Rotation about y-axis           %
%============================================%

% Electrical Motor Dynamics
% --------------------------------------------
% The armature admittance is the electrical component of motor response.
% The motors eletrical characteristics can be modeled with an armature circuit, 
% where the terminal impedance is represented as a resistor and inductor in series.
% Z = sLa + Ra  -->  Y=1/(sLa + Ra)
Ra0 	= Q0(TermR);		 % Terminal (armature) resistance 
La0 	= Q0(TermL)/10^3;	 % Terminal (armature) inductance, mH --> H
Elec0n  = 1;         		 % Numerator: s
Elec0d  = [La0 Ra0];	     % Denominator: sL + R
% --------------------------------------------

% Torque Const & Back EMF
% --------------------------------------------
TConst0  = Q0(TorqueK)/10^3; 		    % Torque constant, mNm/A --> Nm/A
BackEMF0 = 1/(Q0(SpdK)*RadPSecPerRPM);  % SpdK is the speed constant, rpm/v --> to Vs/rad: rpm/V*RadPSecPerRPM = Volt*seconds/rad
% --------------------------------------------

% --------------------------------------------%
% 		Mechanical Motor Dynamics             %
% --------------------------------------------%
% The transfer function of the motor is s/(Js^2 + Bs + K)
% where J is the inertia, B is the damping coefficent, and K is the dynamic friction (spring like behaviour of the motor)
% J units: Nms^2/ra                                                                                                                                 `d
% B units: Nms/rad
% K units: Nm/rad

% J: Moment of Inertia
% --------------------------------------------

% J0Internal = 0.5*massQ0*(shaftRadiusQ0^2 + rotorRadiusQ0^2); 
J0Internal = Q0(RotJ)/10^7;                                            % gcm^2 --> kgm^2: gcm^2*(1kg/1000 g)*(1m^2/100^2cm^2)=kgm^2/10^7
J0EncoderQ0 = 1/2*encoderQ0Mass*encoderQ0Radius^2;

% TODO: PULLEY

J0 = J0Internal + J0EncoderQ0;
% --------------------------------------------

% B: Damping Coefficient
% --------------------------------------------
% STGrad0 = Q0(SpdTorqueGrad)*(10^3)*RadPSecPerRPM;            % SpdTorqueGrad: rpm/mNm --> rpm/Nm --> rad/Nms = STGrad
B0 = Q0(NoLoadCurr)/10^3*Q0(TorqueK)/10^3/(Q0(NoLoadSpd)*RadPSecPerRPM); % NoLoadCurr mA --> A, TorqueK mNm/A --> Nm/A, NoLoadSpd rpm --> rad/s
% B0 = B1;
% --------------------------------------------

% --------------------------------------------

% Mechanical Dynamics Vectors
% --------------------------------------------
Mech0n  = [1];                                             % Numerator: s
Mech0d  = [J0 B0];                                           % Denominator: Js^2 + Bs + K
JntSat0 =  JntLim0*RadPerDeg;                                % Joint saturation set at the angle limit of motor Q0, deg --> rad 
% --------------------------------------------

% Sensor Dynamics
% --------------------------------------------
Sens0    =  1;                                               % Arduino handles conversion to radian
% --------------------------------------------

% Static Friction
% --------------------------------------------
% Fs = us*Ns where us is the coefficient of static friction and Ns is the normal force.
% Motor Q0 controls the ring so the weight associated with the friction is the  
% sum of the weights of the motors and the ring.
% Weight = Weight of Q0 + Weight of Q1 + Weight of ring
% Ns = Weight = Mass*g

% TotalWeight0  = mTotal0*G; 			                         % G is the gravitational acceleration given in CONSTANTS.m
% StFric0      = uSF*TotalWeight0/10^6;                         % Fs = us*Ns, units: N
% --------------------------------------------

%============================================%
% 		Q1 - Rotation about x-axis           %
%============================================%

% Electrical Motor Dynamics
% --------------------------------------------
Ra1 	= Q1(TermR);		                                % Terminal (armature) resistance 
La1 	= Q1(TermL)/10^3;	                                % Terminal (armature) inductance, mH --> H
Elec1n  = 1;         		                                % Numerator: 1
Elec1d  = [La1 Ra1];	                                    % Denominator: sL + R
% ---------------------

% Torque Const & Back EMF
% --------------------------------------------
TConst1  = Q1(TorqueK)/10^3; 					            % Torque constant, mNm/A --> Nm/A
BackEMF1 = 1/(Q1(SpdK)*RadPSecPerRPM); 		   	            % BackEMF Constant (V*s/rad), SpdK is the speed constant (rpm/v --> Vs/rad): rpm/V*RadPSecPerRPM = Volt*seconds/rad
% --------------------------------------------

% Mechanical Motor Dynamics
% --------------------------------------------

% J: Moment of Inertia
% --------------------------------------------
% The total moment of inertia associated with motor Q1 is J1 = J1Internal
% J1Internal is given by just the rotational inertia of Q1, which is a given parameter
% because the mass of the laser is negligible
J1Internal = Q1(RotJ)/10^7;   
% J1Internal = 0.5*massQ1*(shaftRadiusQ1^2 + rotorRadiusQ1^2);                                          % gcm^2 --> kgm^2: gcm^2*(1kg/1000 g)*(1m^2/100^2cm^2)=kgm^2/10^7
J1EncoderQ1 = 1/2*encoderQ1Mass*encoderQ1Radius^2;
J1LaserHolder = 1/2*xholderMass*(xholderDepth^2 + xholderHeight^2);
J1Laser = 1/12*laserMass*(3*laserRadius^2 + laserHeight^2);
J1 = J1Internal + J1EncoderQ1 + J1LaserHolder;
% --------------------------------------------

% B: Damping Coefficient
% --------------------------------------------
% STGrad1 = Q1(SpdTorqueGrad)*10^3*RadPSecPerRPM;                         % Speed Torque Gradent, rpm/mNm --> rpm/Nm --> rad/Nms
B1  = Q1(NoLoadCurr)/10^3*Q1(TorqueK)/10^3/(Q1(NoLoadSpd)*RadPSecPerRPM); % units: Nms/rad
                                                                            % NoLoadCurr mA --> A, TorqueK mNm/A --> Nm/A, NoLoadSpd rpm --> rad/s

% --------------------------------------------


% Mechanical Dynamics Vectors
% --------------------------------------------
Mech1n  = [1];                                          % Numerator: 1
Mech1d  = [J1 B1];		                                % Denominator: Js + B
% Transfer function: 
% Normalized transfer function: 
JntSat1  =  JntLim1*RadPerDeg;                          % Joint saturation set at the angle limit of motor Q1, deg --> rad 
% --------------------------------------------

% Sensor Dynamics
% --------------------------------------------
Sens1    =  1;                                              % Sensor dynamics, units: V/rad, SensAng (deg --> rad)

% --------------------------------------------

% Static Friction
% --------------------------------------------
% There is no static friction for motor Q! because it is in constant motion and the mass of the laser is negligible
% StFric1  = uSF*mTotal1/10^6;     	                                        % Static friction of motor Q1, units: N
% StFric1 = TConst1*200*10^(-3); % 200 mA
StFric1 = 0;
% --------------------------------------------

%============================================%
% 		  Q0 Transfer Functions              
%============================================%
E0	=	tf(Elec0n,Elec0d);                                  % Electrical Motor Dynamics
M0	=	tf(Mech0n,Mech0d);  
% A0 = tf(Amp0n, Amp0d);                                % Mechanical Motor Dynamics

%============================================%
% 		  Q1 Transfer Functions              %
%============================================%
E1	=	tf(Elec1n,Elec1d);                                  % Electrical Motor Dynamics
M1	=	tf(Mech1n,Mech1d); 
% A1 = tf(Amp1n, Amp1d);                                     % Mechanical Motor DynamicsA

% %============================================%
% % 		 Open Loop Gain Calculation          %
% %============================================%

% Integrator
INT = tf(1,[1 0]);

% Q1

GM1 = E1*TConst1*M1;
TM1 = feedback(GM1,BackEMF1);
G1 = MD1_Linear_Gain*TM1;


% Data = load('OL_EXPERIMENTAL.mat');
% plot(Data.data_ol(:,1), Data.data_ol(:,2), 'r');
% % hold on;
% hold on;
% step(G1);
% title('Step Response of Y System Open Loop');
% legend('Linear Motor Driver and Motor');
% legend('Motor Raw', 'Simulink Model', 'Location','southwest');
% ylabel('Response (Rad/s/PWM)'); % x-axis label
% xlabel('Time(s)'); % y-axis label
% hold off;

OL1 = zpk(G1*INT);

% GHPID1 = zpk(tf([96950], [1 2133 0]));
% CL1 = zpk(feedback(GHPID1, 1));

% % CL1_Linear = feedback(G1, 1);
% Data = load('CL_EXPERIMENTAL.mat');
% plot(Data.data_cl(:,1), Data.data_cl(:,2), 'r');
% % hold on;
% hold on;
% step(CL1);
% title('Step Response of Controlled Closed Loop');
% % legend('Linear Motor Driver and Motor');
% legend('Motor Raw', 'Simulink Model', 'Location','southwest');
% ylabel('Response (Rad/Rad)'); % x-axis label
% xlabel('Time(s)'); % y-axis label
% hold off;

% Q0

GM0 = E0*TConst0*M0;
TM0 = feedback(GM0,BackEMF0);
G0 = MD0_Linear_Gain*TM0;

Data = load('X_OL.mat');
plot(Data.x_ol(:,1), Data.x_ol(:,2), 'r');
% hold on;
hold on;
step(G0);
title('Step Response of X System Open Loop');
legend('Linear Motor Driver and Motor');
legend('Motor Raw', 'Simulink Model', 'Location','southwest');
ylabel('Response (Rad/s/PWM)'); % x-axis label
xlabel('Time(s)'); % y-axis label
hold off;

OL0 = zpk(G0*INT);

% GHPID0 = zpk(tf([96950], [0 2033 0]));
% CL0 = zpk(feedback(GHPID0, 0));

% % CL0_Linear = feedback(G0, 0);
% Data = load('CL_EXPERIMENTAL.mat');
% plot(Data.data_cl(:,0), Data.data_cl(:,2), 'r');
% % hold on;
% hold on;
% step(CL0);
% title('Step Response of Controlled Closed Loop');
% % legend('Linear Motor Driver and Motor');
% legend('Motor Raw', 'Simulink Model', 'Location','southwest');
% ylabel('Response (Rad/Rad)'); % x-axis label
% xlabel('Time(s)'); % y-axis label
% hold off;


