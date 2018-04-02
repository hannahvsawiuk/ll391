% This script sets parameter values for the following device:
% Mechanically commutating DC motor

MotorParam = ...
...             % Nominal Values
[0              % NomV          (V)*    % Not measured
 1000           % NoLoadSpd     (rpm)   % Not measured
 820            % NoLoadCurr    (mA)    % Values obtained through measurement
 1354           % NomSpd        (rpm)   % Values obtained through measurement
 0              % NomTorque     (mNm)   % Values obtained from SPEC SHEET
 820            % NomCurr       (mA)    % Values obtained through measurement
 0              % StallTorque   (mNm)   % Not measured
 0              % StallCurr     (A)     % Not measured
 0              % MaxEff        (%)     % Not measured
...
%TODO
...             % Characteristics
 6.05           % TermR         (Ohms)    % Values obtained through measurement
 1.205          % TermL         (mH)      % Values obtained through measurement
 14.388         % TorqueConst   (mNm/A)   % Values obtained through measurement
 664.051        % SpdConst      (rpm/V)   % Values obtained through measurement
 0              % SpdTorqueGrad (rpm/mNm) % Not measured
 0              % MechTimeConst (ms)
 0              % RotJ          (gcm^2)   % Values obtained through measurement
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
 6468           % MaxSpd        (rpm)       % Not measured
 0              % AxialPlay     (mm)
 0              % RadPlay       (mm)
 1              % MaxAxLd       (N)
 0              % MaxF          (N)
 0              % MaxRadLd      (N)
...
...             % Other Specifications
 0              % NoPolePair    (pure)
 0              % NoCommSeg     (pure)
 0              % Weight        (g)
...
...             % Physical Dimensions
 30             % OuterDiam     (mm)        % Not measured
 65];           % Length        (mm)        % Not measured