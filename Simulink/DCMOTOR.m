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
 0              % ThermRhous    (K/W)
 0              % ThermRwind    (K/W)
 0              % ThermTCwind   (s)
 0              % ThermTCmot    (s)
 0              % AmbTemp       (degC)
 0              % MaxTemp       (degC)
...
...             % Mechanical Data
 6468           % MaxSpd        (rpm)       % Values obtained from SPEC SHEET
 0              % AxialPlay     (mm)
 0              % RadPlay       (mm)
 1              % MaxAxLd       (N)
 0              % MaxF          (N)
 0              % MaxRadLd      (N)
...
...             % Other Specifications
 0              % NoPolePair    (pure)
 0              % NoCommSeg     (pure)
 0             % Weight        (g)
...
...             % Physical Dimensions
 30             % OuterDiam     (mm)        % Values obtained from SPEC SHEET
 65];           % Length        (mm)        % Values obtained from SPEC SHEET