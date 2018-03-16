
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
 DCMOTOR;                % Default Maxon motor
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
MDSat1 = 10.8;
InputSat1 = 12;


MD1_Linear_Gain = 0.8920*12/255;

%============================================%
% 			 System Parameters               %
%============================================%
% Some values that will be used in the calculations of the mechanical dynamics

% Mass of system (motors and ring)
% --------------------------------------------
% The values for the motors are given, so just the mass of the ring needs to be calculated.
% Mass = volume*density
% NEED TO CHANGE -> TODO
rIn 	     = LinkR1/10^3;   			        % Inner radius of wrist frame, mm --> m
rOut         = LinkR2/10^3;   			        % Outer radius of wrist frame, mm --> m
wristDepth   = LinkD/10^3;	     		        % Depth of wrist frame, mm --> m
distCentre   = LinkOff/10^3;                    % Distance from motor face to centre, mm --> m
lengthQ0     = Q0(Length)/10^3;                 % Length of motor Q0, mm --> m
lengthQ1     = Q1(Length)/10^3;                 % Length of motor Q1, mm --> m
ringVol      = pi*(rOut^2-rIn^2)*wristDepth;    % Volume of the ring
ringDensity  = RhoAl*10^3;				        % Density of 6061 Al, g/cm^3 --> kg/m^3: g/cm^3*(1kg/1000g)*(100^3cm^3/1m^3)=kg/m^3*10^3      
mRing 	     = ringVol*ringDensity;             % Mass of ring = volume * density
mQ0          = Q0(Weight)/10^3;                 % Mass of motor Q0
mQ1          = Q1(Weight)/10^3;                 % Mass of motor Q1
mTotal0       = 2*mQ1 + mRing;                  % Total mass of loads supported by motor Q0
                                                % Q0 counterweight not included because it is supported by the bar
                                                % Q1 counterweight included because it is supported by motor Q0 and it is assumed ot have the same mass as the motor

mTotal1 = shaftMassQ1;                          % Q1 total mass

xholderMass = 1/1000; %kg
xholderDepth = 7/1000; 
xholderWidth = 18.26/1000;
xholderHeight = 10/1000;

laserRadius = 3.5*10^(-3);
laserHeight = 21*10^(-3);
laserMass = 2*10^(-3);

% --------------------------------------------


encoderQ1Mass = 5.6*10^(-3); % mass in kg: experimentally obtained
encoderQ1Radius = 24*10^(-3); % m: from datasheet
encoderQ1Height = 1.63*10^(-3); %m: measured using calipers

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
% J units: Nms^2/rad
% B units: Nms/rad
% K units: Nm/rad

% J: Moment of Inertia
% --------------------------------------------
% The total moment of inertia associated with motor Q0 is J0 = J0Ring + J0Internal + J0MotorQ1
% J0Ring is the moment of inertia of the ring, J0Internal is the moment of inertia of motor Q0,
% and J0MotorQ1 is the moment of inertia of motor Q1 with respect to motor Q0

% J0Internal: rotational inertia of Q0, which is a given parameter.
J0Internal = Q0(RotJ)/10^7;    % Internal Motor Inertia, gcm^2 --> kgm^2: gcm^2*(1kg/1000 g)*(1m^2/100^2cm^2)=kgm^2/10^7

% J0Ring: the moment of inertia (about y axis) for a hollow cylinder about the y-axis is given by:
dIn    = 2*rIn;      				                          % Inner diameter of wrist frame
dOut   = 2*rOut; 	  				                          % Outer radius of wrist frame
J0Ring = (1/4)*mRing*((dIn^2+dOut^2)/4 + (wristDepth^2)/3);   % Inertia of Ring, Moment of Inertia Calculation for  a Hollow Ring

% J0MotorQ1: the moment of inertia of motor Q1 (about y axis) with respect to motor Q0
% The inertia will be calculated for two cylinders separately. One cylinder will extend from the end of the motor Q1 to the centre
% of the ring (Cylinder 1). The other cylinder will extend from the start of motor Q1 to the centre of the ring. The substraction 
% of these inertias will give the inertia of a the motor Q1 without the counterweight. By assuming that the counterweight has the 
% same mass as motor Q1, this result can be multiplied by 2 to get the total inertia of the motor Q1 system about the y axis.
M10 = mQ1 + (distCentre/lengthQ1)*mQ1;                       % Mass of Cylinder 1 (calculated assuming uniform density in motor Q1)
M20 = (distCentre/lengthQ1)*mQ1                              % Mass of Cylinder 2 (calculated assuming uniform density in motor Q1)
% Inertia about y axis with respect to the end of a cylinder
% Iend = mL^2/3
J10 = M10*(lengthQ1 + distCentre)^2/3;                       % Inertia of Cylinder 1 about the y-axis
J20 = M20*(distCentre)^2/3;                                  % Inertia of Cylinder 2 about the y-axis
J0MotorQ1 = (J10 - J20) * 2;                                 % Total inertia of motor Q1 system about the y axis

% J0
% The mass of the parts in which the ring and the motor Q1 system overlap will be accounted for
% twice, but this is negligible
J0 = J0Ring + J0Internal + J0MotorQ1;			             % units: Nms^2/rad
% --------------------------------------------

% B: Damping Coefficient
% --------------------------------------------
% STGrad0 = Q0(SpdTorqueGrad)*(10^3)*RadPSecPerRPM;            % SpdTorqueGrad: rpm/mNm --> rpm/Nm --> rad/Nms = STGrad
B0 = Q0(NoLoadCurr)/10^3*Q0(TorqueK)/10^3/(Q0(NoLoadSpd)*RadPSecPerRPM); % NoLoadCurr mA --> A, TorqueK mNm/A --> Nm/A, NoLoadSpd rpm --> rad/s

% --------------------------------------------

% --------------------------------------------

% Mechanical Dynamics Vectors
% --------------------------------------------
Mech0n  = [1 0];                                             % Numerator: s
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

TotalWeight0  = mTotal0*G; 			                         % G is the gravitational acceleration given in CONSTANTS.m
StFric0      = uSF*TotalWeight0/10^6;                         % Fs = us*Ns, units: N
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
J1Internal = Q1(RotJ)/10^7;                                             % gcm^2 --> kgm^2: gcm^2*(1kg/1000 g)*(1m^2/100^2cm^2)=kgm^2/10^7
% J1ShaftQ1 = (shaftMassQ1/12)*(3*(shaftRadiusQ1)^2 + shaftLengthQ1^2);   % Inertia for Q1 shaft
% J1 = J1Internal + J1ShaftQ1;			                                  % Total Inertia for motor Q1, units: Nms^2/rad
% J1EncoderQ1 = encoderQ1Mass/12*(3*encoderQ1Radius^3 + encoderQ1Height^2); % Inertia for Q1 encoder
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
% T1_1 = feedback(M1,StFric1);
GM1 = E1*TConst1*M1;
TM1 = feedback(GM1,BackEMF1);
% GH1 = A1*T1_1*INT;
% GH1 = zpk(GH1);
% KDC1 = dcgain(GH1);
% OL1 = feedback(GH1,1);


% subplot(2,1,1);
% step(TM1);
% subplot(2,1,2);
% Mx1 = linspace(0,0.001,0.3);
% My1 = -69076*x.^2 + 4512*x;
% plot(My1, Mx1);


% TM1_EXP = @(t) (-384313*t.^6+484989*t.^5-207646*t.^4+33326*t.^3-1489*t.^2+275.46*t-0.88);
% t = 0:0.001:0.6;
% plot(t,TM1_EXP(t), 'm');
% grid on;
% hold on;
% TM2_EXP = @(x) (48955*x.^6-107809*x.^5+91032*x.^4-35718*x.^3+5807.2*x.^2-19.845*x+1.5973);
% x = 0:0.001:0.6;
% plot(x,TM2_EXP(x), 'c');
% grid on;
% hold on;
% Data = load('TM_EXPERIMENTAL.mat');
% plot(Data.data_x(:,1), Data.data_x(:,2), 'r');
% step(TM1);
% hold on;
% title('Step Response of Motor Closed Loop');
% legend('Motor Smooth', 'Motor Raw', 'Simulink Model', 'Location','southwest');
% ylabel('Response (Rad/S/V)'); % x-axis label
% xlabel('Time(s)'); % y-axis label
% hold off;

% TM1_EXP = @(t) (955591*t.^6-1*10^6*t.^5+715515*t.^4-181763*t.^3+20369*t.^2-410.82*t+2.1082);
% t = 0:0.001:0.6;
% plot(t,TM1_EXP(t), 'm');
% grid on;
% hold on;
% TM2_EXP = @(x) (48955*x.^6-107809*x.^5+91032*x.^4-35718*x.^3+5807.2*x.^2-19.845*x+1.5973);
% x = 0:0.001:0.6;
% plot(x,TM2_EXP(x), 'c');
% grid on;
% hold on;
% Data = load('TM_EXPERIMENTAL2.mat');
% plot(Data.data(:,1), Data.data(:,2), 'r');
% hold on;
% step(TM1);
% hold on;
% title('Step Response of Motor Closed Loop');
% % legend('Motor Smooth', 'Motor Raw', 'Simulink Model', 'Location','southwest');
% legend('Motor Raw', 'Simulink Model', 'Location','southwest');
% ylabel('Response (Rad/S/V)'); % x-axis label
% xlabel('Time(s)'); % y-axis label
% hold off;


G1 = MD1_Linear_Gain*TM1;

% % CL1_Linear = feedback(G1, 1);
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
% Ol1 = zpk(OL1);




% % Without static friction
% G0_1 = E0*TConst0*M0;
% T0_1 = feedback(G0_1,BackEMF0);
% GH0 = A0*T0_1*INT;
% GH0 = zpk(GH0);
% KDC0 = dcgain(GH0);
% T0 = feedback(GH0,1);

% % With static friction


% % T0_2 = feedback(M0,StFric0);
% % G0_1 = E0*TConst0*T0_2;
% % T0_1 = feedback(G0_1,BackEMF0);
% % GH0 = A0*T0_1*INT;
% % GH0 = zpk(GH0);
% % KDC0 = dcgain(GH0);
% % T0 = feedback(GH0,1);


% % Q1
% G1_1 = E1*TConst1*M1; 
% T1_1 = feedback(G1_1,BackEMF1);
% % T1_1 = G1_1/(1+G1_1*BackEMF1);
% GH1 = A1*T1_1*INT;
% GH1 = zpk(GH1);
% T1 = feedback(GH1,1);
% KDC1 = dcgain(GH1);
% % disp(stepinfo(T1_1));
% % disp(stepinfo(GH1));

% % PID Transfer Functions
% GHPID0 = zpk(tf([1.282e08],[1 15279.1702128 748862.340426 0]));
% GHPID1 = zpk(tf([1.4146e10],[1 40450.60 2044240 0]));

GHPID1 = zpk(tf([96950], [1 2133 0]));
CL1 = zpk(feedback(GHPID1, 1));

% CL1_Linear = feedback(G1, 1);
Data = load('CL_EXPERIMENTAL.mat');
plot(Data.data_cl(:,1), Data.data_cl(:,2), 'r');
% hold on;
hold on;
step(CL1);
title('Step Response of Controlled Closed Loop');
% legend('Linear Motor Driver and Motor');
legend('Motor Raw', 'Simulink Model', 'Location','southwest');
ylabel('Response (Rad/Rad)'); % x-axis label
xlabel('Time(s)'); % y-axis label
hold off;

OL1 = zpk(G1*INT);
% Ol1 = zpk(OL1);



