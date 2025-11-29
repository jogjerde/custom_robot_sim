% Felles parametere for truck + arm (meter)
params = struct();

% Mobil base (ackermann)
params.L_car   = 1.0;   % akselavstand ("1 meter lang bil")
params.v_max   = 0.8;   % m/s, maks sim-hastighet
params.dt      = 0.05;  % tidssteg

% Alternativ diff-drive (for diff-drive plugin i Gazebo)
params.W_track = 0.60;  % sporvidde (avstand mellom drivhjul)

% Arm-geometri (yaw + 3x pitch i samme plan)
params.h_base  = 1.00;  % basehøyde fra gulv til J2-plan
params.a2      = 1.00;  % første lenke (kravet deres: 1 m)
params.a3      = 1.50;  % andre lenke (gir god vertikal rekkevidde)
params.a4      = 1.20;  % verktøy-offset (kan settes 0)
