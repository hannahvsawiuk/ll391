% This script sets parameter values for the following device:
% Mechanically commutating DC motor

MotorParam = ...
...             % Nominal Values
[12             % NomV          (V)*    % Values obtained from SPEC SHEET
 7200           % NoLoadSpd     (rpm)   % Values obtained from SPEC SHEET
 190            % NoLoadCurr    (mA)    % Values obtained from SPEC SHEET
 7060           % NomSpd        (rpm)   % Values obtained from SPEC SHEET
 12.0425662     % NomTorque     (mNm)   % Values obtained from SPEC SHEET
 930            % NomCurr       (mA)    % Values obtained from SPEC SHEET
 91.005712      % StallTorque   (mNm)   % Values obtained from SPEC SHEET
 6.031          % StallCurr     (A)     % Values obtained from SPEC SHEET
 74             % MaxEff        (%)     % Values obtained from SPEC SHEET
...
%TODO
...             % Characteristics
 5.53           % TermR         (Ohms)
 0.363          % TermL         (mH)
 10.9           % TorqueConst   (mNm/A)
 875            % SpdConst      (rpm/V)
 444            % SpdTorqueGrad (rpm/mNm)
 19.9           % MechTimeConst (ms)
 4.29           % RotJ          (gcm^2)   *
...
...             % Thermal Data
 20             % ThermRhous    (K/W)
 6.0            % ThermRwind    (K/W)
 10.2           % ThermTCwind   (s)
 314            % ThermTCmot    (s)
 27.5           % AmbTemp       (degC)
 125            % MaxTemp       (degC)
...
...             % Mechanical Data
 9800           % MaxSpd        (rpm)
 0.1            % AxialPlay     (mm)
 0.012          % RadPlay       (mm)
 1              % MaxAxLd       (N)
 80             % MaxF          (N)
 2.8            % MaxRadLd      (N)
...
...             % Other Specifications
 1              % NoPolePair    (pure)
 9              % NoCommSeg     (pure)
 54             % Weight        (g)
...
...             % Physical Dimensions
 22             % OuterDiam     (mm)
 32];           % Length        (mm)