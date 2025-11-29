clear; clc; close all
run forklift_parameter.m
h=params.h_base; a2=params.a2; a3=params.a3; a4=params.a4;

% --- Definer arm (RTB10) ---
L1 = Link('d', h,  'a', 0,  'alpha',  pi/2);
L2 = Link('d', 0,  'a', a2, 'alpha',  0   );
L3 = Link('d', 0,  'a', a3, 'alpha',  0   );
L4 = Link('d', 0,  'a', a4, 'alpha',  0   );
robot = SerialLink([L1 L2 L3 L4], 'name', 'ForkliftArm');
robot.qlim = [ -pi  pi;   -pi/2  pi/2;   -3*pi/4  3*pi/4;   -3*pi/4  3*pi/4 ];

% --- Målposisjoner (base_link: x frem, y venstre, z opp) ---
T_pick  = transl(2.20, 0.00, -0.90);   % foran pall (i gang)
T_liftup = transl(2.20, 0.00, -0.70);   % hylleplass (lavt nivå)
T_back = transl(1.00, 0.00, -0.70);   % hylleplass (lavt nivå)
T_up = transl(1.00, 0.00, 0.00);   % hylleplass (lavt nivå)
T_forward = transl(2.20, 0.00, 1.00);   % hylleplass (lavt nivå)
T_place = transl(2.20, 0.00, 0.80);   % hylleplass (lavt nivå)
T_disengage = transl(0.80, 0.00, 0.80);   % hylleplass (lavt nivå)

% --- IK i RTB10: bruk POSISJON-KUN maske ---
q0    = [0 0 0 0];
maskP = [1 1 1 0 0 0];                 % <— viktig: ikke lås orientasjon

% Robust IK (RTB10-syntaks) med stram toleranse og høy iterasjonsgrense
q_home = [0 0 0 0];
q_pick  = robot.ikine(T_pick,  'q0', q_home,     'mask', maskP, 'tol', 1e-9, 'ilimit', 2000);
q_liftup  = robot.ikine(T_liftup,  'q0', q_pick,     'mask', maskP, 'tol', 1e-9, 'ilimit', 2000);
q_back = robot.ikine(T_back, 'q0', q_liftup, 'mask', maskP, 'tol', 1e-9, 'ilimit', 2000);
q_forward = robot.ikine(T_forward, 'q0', q_back, 'mask', maskP, 'tol', 1e-9, 'ilimit', 2000);
q_place = robot.ikine(T_place, 'q0', q_forward, 'mask', maskP, 'tol', 1e-9, 'ilimit', 2000);
q_disengage = robot.ikine(T_disengage, 'q0', q_place, 'mask', maskP, 'tol', 1e-9, 'ilimit', 2000);


% Sjekk/ev. fallback hvis IK ikke konvergerer
if any(isnan(q_pick))
    warning('IK for T_{pick} konvergerte ikke – bruker lukket formel som start.');
    q_pick = ik4dof_level(T_pick, h, a2, a3, a4);
end
if any(isnan(q_liftup))
    warning('IK for T_{place} konvergerte ikke – bruker lukket formel som start.');
    q_liftup = ik4dof_level(T_place, h, a2, a3, a4);
end
if any(isnan(q_back))
    warning('IK for T_{place} konvergerte ikke – bruker lukket formel som start.');
    q_back = ik4dof_level(T_place, h, a2, a3, a4);
end
if any(isnan(q_forward))
    warning('IK for T_{place} konvergerte ikke – bruker lukket formel som start.');
    q_forward = ik4dof_level(T_place, h, a2, a3, a4);
end
if any(isnan(q_place))
    warning('IK for T_{place} konvergerte ikke – bruker lukket formel som start.');
    q_place = ik4dof_level(T_place, h, a2, a3, a4);
end
if any(isnan(q_disengage))
    warning('IK for T_{place} konvergerte ikke – bruker lukket formel som start.');
    q_disengage = ik4dof_level(T_place, h, a2, a3, a4);
end

% --- Hold gaffel vannrett i endepunktene ---
q_pick(4)  = -(q_pick(2)  + q_pick(3));
q_place(4) = -(q_place(2) + q_place(3));

% --- Trajektorier (viktig: re-kompenser q4 langs hele banen) ---
q1 = jtraj(q_home, q_pick,  40);
q2 = jtraj(q_pick, q_liftup, 40);
q3 = jtraj(q_liftup, q_back, 40);
q4 = jtraj(q_back, q_forward, 40);
q5 = jtraj(q_forward, q_place, 40);
q6 = jtraj(q_place, q_disengage, 40);
q7 = jtraj(q_disengage, q_home, 40);


q1 = apply_level(q1);
q2 = apply_level(q2);
q3 = apply_level(q3);
q4 = apply_level(q4);
q5 = apply_level(q5);
q6 = apply_level(q6);
q7 = apply_level(q7);

q_traj = [q1; q2; q3; q4; q5; q6; q7];

% --- Visualiser ---
figure('Name','IK pick->place with level forks')
w = [-0.5 5 -1.5 1.5 0 3.5];
robot.plot(q_traj, 'workspace', w);


% ======= HJELPEFUNKSJONER =======
function Q = apply_level(Q)
    % Sørger for at q4 = -(q2+q3) i hele banen
    for i = 1:size(Q,1)
        Q(i,4) = -(Q(i,2) + Q(i,3));
    end
end

function q = ik4dof_level(T, h, a2, a3, a4)
    % Lukket IK for 4-DOF (yaw + 2 ledd i plan + horisontal-kompensasjon)
    p = transl(T);  x = p(1); y = p(2); z = p(3);
    q1 = atan2(y,x);             % yaw peker mot målet
    r  = hypot(x,y) - a4;        % projeksjon i armplanet
    zc = z - h;                  % høyde relativt J2
    c3 = (r^2 + zc^2 - a2^2 - a3^2) / (2*a2*a3);
    if abs(c3) > 1, error('Mål utenfor rekkevidde'); end
    s3 = sqrt(1 - c3^2);
    q3 = atan2(s3, c3);          % albue-ned
    q2 = atan2(zc, r) - atan2(a3*s3, a2 + a3*c3);
    q4 = -(q2 + q3);             % hold gaffel horisontal
    q  = [q1 q2 q3 q4];
end
