   Kp0 = 1;
    Ki0 = 0;
    Kd0 = 0;
ctrl0 = pid(Kp0,Ki0,Kd0, 1/100);
    Kp1 = 1;
    Ki1 = 0;
    Kd1 = 0;
ctrl1 = pid(Kp1,Ki1,Kd1, 1/100);
    
PID0 = [Kp0 Ki0 Kd0];
PID1 = [Kp1 Ki1 Kd1];
%% Enter feedback sensor values here.
FB0 = 1/Sens0;   % to get actual value in [rad] from the sensor 
FB1 = 1/Sens1;   % to get actual value in [rad] from the sensor 
%% Calculate the COMMON blocks in both joints
    AMP_tf = tf(Amp0n, Amp0d);
    integrator_tf = tf(1,[1,0]);
    
%% Calculate the Joint 0 & 1 Elec & mech feedback loop 
    Elec_dyn_tf0 = tf(Elec0n, Elec0d);
    Mech_dyn_tf0 = tf(Mech0n,Mech0d);
    G_motor0 = Elec_dyn_tf0 * TConst0 * Mech_dyn_tf0;
    H_motor0 = BackEMF0;
Motor0_feedback_tf = feedback(G_motor0 , H_motor0); 
    Elec_dyn_tf1 = tf(Elec1n, Elec1d);
    Mech_dyn_tf1 = tf(Mech1n,Mech1d);
    G_motor1 = Elec_dyn_tf1 * TConst1 * Mech_dyn_tf1;
    H_motor1 = BackEMF1;
Motor1_feedback_tf = feedback(G_motor1 , H_motor1);
    
%% Calculate the Joint 0 & 1 total feedback loop (CL_tf) and OL_tf
    G_total0 = AMP_tf * Motor0_feedback_tf * integrator_tf;
    H_total0 = FB0 * Sens0;
CL_tf0 = feedback(ctrl0 * G_total0 , H_total0); 
OL_tf0 = ctrl0 * G_total0 *H_total0;
    
    
    G_total1 = AMP_tf * Motor1_feedback_tf * integrator_tf;
    H_total1 = FB1 * Sens1;
CL_tf1 = feedback(ctrl1 * G_total1 , H_total1); 
OL_tf0 = ctrl1 * G_total1 * H_total1;
    