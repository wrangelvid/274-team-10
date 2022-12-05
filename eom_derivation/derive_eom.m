%% Derive Equations of Motion and Control Law Equations
clear;clc
name = 'arm';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t real                                             % time

syms th1 dth1 ddth1 th2 dth2 ddth2 th3 dth3 ddth3 real  % joint angles
syms tau1 tau2 real                                     % control vars

syms m1A m1B m2A m2B m3 mfb1 real                       % masses
syms I1A I1B I2A I2B I3 Ifb1 Ir real                    % moments of inertia (Izz)
syms l1 l2 l3 lfb0 lfb1 d1 d2 d3 real                   % joint-to-joint lengths
syms lm1A lm1B lm2A lm2B lm3 lmfb1 real                 % CoM distances
syms lb1A lb1B lb2A lb2B real                           % acrylic bar start/stop distances
syms N lam_bars rho_fb2 k d_shaft wfb2 lsp0 real        % other wacky parameters

% Group them
q   = [  th1;   th2;   th3];                            % generalized coordinates
dq  = [ dth1;  dth2;  dth3];                            % first time derivatives
ddq = [ddth1; ddth2; ddth3];                            % second time derivatives
u   = [ tau1; tau2];                                    % controls
p   = [...                                              % parameters
    m1A m1B m2A m2B m3 mfb1 ...
    I1A I1B I2A I2B I3 Ifb1 Ir ...
    l1 l2 l3 lfb0 lfb1 d1 d2 d3 ...
    lm1A lm1B lm2A lm2B lm3 lmfb1 ...
    lb1A lb1B lb2A lb2B ...
    N lam_bars rho_fb2 k d_shaft wfb2 lsp0]';

% Pre-calculate additional properties
bars1 = l1-lb1A-lb1B;
mbars1 = bars1*lam_bars;

bars2 = l2-lb2A-lb2B;
mbars2 = bars2*lam_bars;

r_shaft = d_shaft/2;
lfb2 = lfb0+l1;
lmfb2 = lfb2/2;
mfb2 = (lfb2*wfb2 + pi*(wfb2/2)^2 - 2*pi*r_shaft^2)*rho_fb2;
Ifb2 = rho_fb2 * (1/12*lfb2*wfb2*(lfb2^2+wfb2^2) + pi*(wfb2/2)^2 * ((lfb2/2)^2 + 1/2*(wfb2/2)^2) - 2*pi*r_shaft^2 * ((lfb2/2)^2 + 1/2*pi*r_shaft^2));

% Generate Vectors and Derivatives
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = [0; 0; 1];

er1hat =  sin(th1        )*ihat + cos(th1        )*jhat;
er2hat =  sin(th1+th2    )*ihat + cos(th1+th2    )*jhat;
er3hat =  sin(th1+th2+th3)*ihat + cos(th1+th2+th3)*jhat;

ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; % a handy anonymous function for taking time derivatives
magsq = @(r) simplify(dot(r,r));                  % function to find the magnitude squared of a vector

% Joint positions/velocities
rA =      l1*er1hat;
rB = rA + l2*er2hat;
rC1 =     - lfb0*er1hat;
rC2 = rC1 + lfb1*er2hat;
rC3 = rA  + lfb1*er2hat;
rS0 = rB - d1*er2hat;
rS1 = rS0 + d3*rotz( 90)*er2hat;
rS2 = rS0 + d3*rotz(-90)*er2hat;
rT0 = rB + d2*er3hat;
rT1 = rT0 + d3*rotz( 90)*er3hat;
rT2 = rT0 + d3*rotz(-90)*er3hat;
rE = rB + l3*er3hat;

drE = ddt(rE);

% CoM positions/velocities
rm1A =      lm1A*er1hat;
rm1B = rA + lm1B*er1hat;
rm1bars = rA + (lb1A + bars1/2)*er1hat;
rm1 = (m1A*rm1A + mbars1*rm1bars + m1B*rm1B) / (m1A + mbars1 + m1B);

rm2A = rA + lm2A*er2hat;
rm2B = rB + lm2B*er2hat;
rm2bars = rB + (lb2A + bars2/2)*er2hat;
rm2 = (m2A*rm2A + mbars2*rm2bars + m2B*rm2B) / (m2A + mbars2 + m2B);

rm3 = rB + lm3*er3hat;
rmfb1 = rC1 + lmfb1*er2hat;
rmfb2 = rC2 + lmfb2*er1hat;

drm1 = ddt(rm1);
drm2 = ddt(rm2);
drm3 = ddt(rm3);
drmfb1 = ddt(rmfb1);
drmfb2 = ddt(rmfb2);

% Aggregate interia properties
m1 = m1A + mbars1 + m1B;
m2 = m2A + mbars2 + m2B;

I1 = m1A*magsq(rm1-rm1A) + mbars1*magsq(rm1-rm1bars) + m1B*magsq(rm1-rm1B);
I2 = m2A*magsq(rm2-rm2A) + mbars2*magsq(rm2-rm2bars) + m2B*magsq(rm2-rm2B);

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces

T1 = 1/2*m1*magsq(drm1) + 1/2*I1*dth1^2;
T2 = 1/2*m2*magsq(drm2) + 1/2*I2*(dth1+dth2)^2;
T3 = 1/2*m3*magsq(drm3) + 1/2*I3*(dth1+dth2+dth3)^2;
Tfb1 = 1/2*mfb1*magsq(drmfb1) + 1/2*Ifb1*(dth1+dth2)^2;
Tfb2 = 1/2*mfb2*magsq(drmfb2) + 1/2*Ifb2*dth1^2;
T1r = (1/2)*Ir*(N*dth1)^2;
T2r = (1/2)*Ir*(dth1 + N*dth2)^2;

Vsp1 = 1/2*k*(norm(rT1-rS1)-lsp0)^2;
Vsp2 = 1/2*k*(norm(rT2-rS2)-lsp0)^2;

T = simplify(T1 + T2 + T3 + Tfb1 + Tfb2 + T1r + T2r);
V = Vsp1 + Vsp2;
Q_tau1 = M2Q(tau1*khat,dth1*khat);
Q_tau2 = M2Q(tau2*khat,dth2*khat);
Q = Q_tau1 + Q_tau2;

% Assemble the array of cartesian coordinates of the key points
keypoints = [rA rB rC1 rC2 rC3 rS0 rS1 rS2 rT0 rT1 rT2 rE];
keypoints = keypoints(1:2, :);

% Derive Energy Function and Equations of Motion
E = T+V;
L = T-V;
g = ddt(jacobian(L,dq).') - jacobian(L,q).' - Q;

% Rearrange Equations of Motion
A = jacobian(g,ddq);
b = A*ddq - g;

% Export Energy Function and Equations of Motion
z  = [q ; dq];
matlabFunction(A,'file',['out/A_' name],'vars',{z p});
matlabFunction(b,'file',['out/b_' name],'vars',{z u p});
matlabFunction(E,'file',['out/energy_' name],'vars',{z p});
matlabFunction(keypoints,'file',['out/keypoints_' name],'vars',{z p});

% Export end effector jacobian
JE = jacobian(rE(1:2), q);
matlabFunction(JE, 'file', ['out/JE_' name], 'vars', {z p});

% Export inertia properties
matlabFunction([I1;I2;Ifb2],'file',['out/I_' name],'vars',{z p});
matlabFunction([m1;m2;mfb2],'file',['out/m_' name],'vars',{z p});
matlabFunction([simplify(norm(rm1));simplify(norm(rm2-rA))],'file',['out/lm_' name],'vars',{z p});
