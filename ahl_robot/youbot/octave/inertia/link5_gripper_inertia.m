% Assume that all frames' rotation matrix is identity matrix.

Ml = 0.68700
% Inertia matrix of gripper link in frame attached to its COM
Il_cl = ...
  [6.8900e-05   1.1181e-20   7.6234e-21; ...
   1.1181e-20   1.6020e-04  -9.3360e-37; ...
   7.6234e-21  -9.3360e-37   1.9340e-04];
% COM of link5 in frame attached to link5
Pl_l = [0; -0.0012; -0.01648; 1];

% Add grippers weight to gripper link
Mg = 0.19900 + 0.01 + 0.01
% Inertia matrix of gripper link in frame attached to its COM
Ig_cg = ...
  [ 2.3240e-04  -3.1473e-21  -1.1713e-36; ...
   -3.1473e-21   2.0670e-04  -9.5645e-21; ...
   -1.1713e-36  -9.5645e-21   3.6290e-04];
% COM of gripper link in frame attached to gripper link
Pg_g = [0; 0; 0.0289; 1];

Tlg = ...
  [1 0 0      0; ...
   0 1 0      0; ...
   0 0 1 0.0965; ...
   0 0 0      1];

% Pg_l is COM of gripper in frame attached to link5
Pg_l = Tlg * Pg_g;
Pg_l = Pg_l(1:3);

I = [1 0 0; 0 1 0; 0 0 1];
% Ig_ol is inertia matrix w.r.t frame attached to origin of link5
Ig_ol = Ig_cg + Mg * ((Pg_l.' * Pg_l) * I - Pg_l * Pg_l.');

Pl_l = Pl_l(1:3);
% Ig_cl is inertia matrix w.r.t frame attached to COM of link5
M = Ml + Mg
Pc_l = (Ml / M) * Pl_l + (Mg / M) * Pg_l
Il_cl
Ig_cl = Ig_ol + Mg * ((Pc_l.' * Pc_l) * I - Pc_l * Pc_l.')
I_cl = Il_cl + Ig_cl
