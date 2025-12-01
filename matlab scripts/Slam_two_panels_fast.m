function Slam_two_panels_fast
%% To-panel visning + raskere SLAM-skanning
% Krav: Navigation Toolbox + Lidar Toolbox
clear; clc; close all; rng(0);

%% ----- Parametre (skru opp/ned her ved behov) ------------------------------
worldW = 20; worldH = 14;
res    = 12;                 % celler/m (litt grovere -> raskere)
dt     = 0.05;               % 20 Hz simulasjon
vRef   = 1.0;                % m/s (kjør litt fortere)
lookahead = 0.9;             % m
mapInflate = 0.05;           % m

% LiDAR (raskere):
lidarRangeMax = 8;           % m (kortere -> raskere SLAM)
angleResDeg   = 2;           % 2° (≈180 stråler)

% Hvor ofte å tegne / bygge SLAM-kart
plotEvery = 8;               % hvert 8. steg (~0.4 s)
mapEvery  = 16;              % bygg SLAM-kart sjeldnere (~0.8 s)

%% ----- 1) Kart m/reoler (top-down) -----------------------------------------
mapGT = occupancyMap(zeros(round(worldH*res), round(worldW*res)), res);

% yttervegger
drawWall(mapGT, [0 0],        [worldW 0]);
drawWall(mapGT, [worldW 0],   [worldW worldH]);
drawWall(mapGT, [worldW worldH],[0 worldH]);
drawWall(mapGT, [0 worldH],   [0 0]);

% reoler som blokker + tverrganger
shelfX      = [4 10 16];
shelfYmin   = 2;  shelfYmax = 12;
shelfWidth  = 0.6;
crossY      = [4.5 8.5];
crossH      = 1.2;

for k = 1:numel(shelfX)
    xL = shelfX(k) - shelfWidth/2;
    xR = shelfX(k) + shelfWidth/2;
    fillRectWorld(mapGT, [xL shelfYmin xR shelfYmax], 1);
    for gy = crossY
        yL = gy - crossH/2;
        yU = gy + crossH/2;
        clearRectWorld(mapGT, [xL-0.05 yL xR+0.05 yU]);
    end
end
inflate(mapGT, mapInflate);

%% ----- 2) Waypoints + Pure Pursuit -----------------------------------------
wps = [ 1.0  1.0;  18.5 1.0;   18.5 4.5;  1.5 4.5; ...
        1.5  8.5;  18.5 8.5;  18.5 13.0; 1.0 13.0; ...
        1.0  1.0 ];

ctrl = controllerPurePursuit;
ctrl.Waypoints             = wps;
ctrl.LookaheadDistance     = lookahead;
ctrl.DesiredLinearVelocity = vRef;
ctrl.MaxAngularVelocity    = 2.0;

%% ----- 3) LiDAR-sensor (raskere) -------------------------------------------
lidar = rangeSensor;
lidar.Range           = [0.25 lidarRangeMax];
lidar.HorizontalAngle = [-pi pi];
if isprop(lidar,'HorizontalAngleResolution')
    lidar.HorizontalAngleResolution = deg2rad(angleResDeg);
elseif isprop(lidar,'HorizontalResolution')
    lidar.HorizontalResolution = round(360/angleResDeg);
end

%% ----- 4) SLAM --------------------------------------------------------------
slam = lidarSLAM(res, lidar.Range(2));
slam.LoopClosureThreshold    = 120;
slam.LoopClosureSearchRadius = 2.0;

%% ----- 5) Simulering (kun 2 paneler) ---------------------------------------
poseTrue = [1.0 1.0 0];
poseOdom = poseTrue;

% bootstrap
[z0, a0] = lidar(poseTrue, mapGT);        % NB: (pose, map) i din versjon
addScan(slam, lidarScan(z0, a0));

% fig med 1x2 paneler
figure('Name','SLAM demo (to paneler)');
tiledlayout(1,2, 'Padding','compact', 'TileSpacing','compact');

% Venstre panel: fasit + bane
nexttile(1); cla
show(mapGT); hold on; grid on
hTrue = plot(poseTrue(1), poseTrue(2), 'LineWidth', 1.2);
plot(wps(:,1), wps(:,2), 'k--o', 'MarkerSize', 3, 'LineWidth', 1);
xlim([0 worldW]); ylim([0 worldH]); axis equal
title('Fasit-kart (top‑down reoler) og sann bane')
xlabel('X [meters]'); ylabel('Y [meters]');

% Høyre panel: SLAM-kart
nexttile(2); cla
title('SLAM-kart'); grid on; axis equal
xlim([-10 worldW+5]); ylim([-10 worldH+5]);

tEnd = 80;
k = 0;
for t = 0:dt:tEnd
    k = k + 1;

    % Pure Pursuit
    [v, w] = ctrl(poseTrue);
    if norm(poseTrue(1:2) - wps(end,:)) < 0.4
        v = 0; w = 0;
    end

    % Kinematikk (sann + odometri)
    th = poseTrue(3);
    poseTrue = poseTrue + [v*cos(th) v*sin(th) w]*dt;

    vN = v*(1+0.01*randn);  wN = w + 0.01*randn;
    thO = poseOdom(3);
    poseOdom = poseOdom + [vN*cos(thO) vN*sin(thO) wN]*dt;

    % LiDAR + SLAM (raskt)
    [rngs, angs] = lidar(poseTrue, mapGT);
    isAccepted = addScan(slam, lidarScan(rngs, angs), poseOdom(1:3)); %#ok<NASGU>

    % Oppdater plott sjeldnere for fart
    if mod(k, plotEvery) == 0
        % venstre: bane
        set(hTrue, 'XData', [get(hTrue,'XData') poseTrue(1)], ...
                   'YData', [get(hTrue,'YData') poseTrue(2)]);

        % høyre: bygg/vis SLAM-kart sjeldnere
        if mod(k, mapEvery) == 0
            [scansUsed, slamPoses] = scansAndPoses(slam);
            slamMap = tryBuildMap(scansUsed, slamPoses, res, lidar.Range(2), slam);
            nexttile(2); cla
            if ~isempty(slamMap)
                show(slamMap); hold on; grid on
            else
                plot(slamPoses(:,1), slamPoses(:,2), '.', 'MarkerSize', 6); hold on; grid on
            end
            if ~isempty(slamPoses)
                plot(slamPoses(:,1), slamPoses(:,2), '-', 'LineWidth', 1.2);
            end
            axis equal; xlim([-10 worldW+5]); ylim([-10 worldH+5]);
            title(sprintf('SLAM-kart (n=%d scans)', size(slamPoses,1)));
        end
        drawnow limitrate
    end
end

end % main


%% ---------------- HJELPEFUNKSJONER ----------------
function drawWall(map, p1, p2)
    step = 0.4 / map.Resolution;
    n = max(2, ceil(norm(p2 - p1)/step));
    xs = linspace(p1(1), p2(1), n)'; ys = linspace(p1(2), p2(2), n)';
    setOccupancy(map, [xs ys], 1);
end

function fillRectWorld(map, rect, occ)
% rect = [xmin ymin xmax ymax], verdenskoordinater
    if nargin < 3, occ = 1; end
    x1 = rect(1); y1 = rect(2); x2 = rect(3); y2 = rect(4);
    if x2 < x1, [x1,x2] = deal(x2,x1); end
    if y2 < y1, [y1,y2] = deal(y2,y1); end
    step = 1 / map.Resolution;     % ≈ cellestørrelse
    xs = x1:step:x2; ys = y1:step:y2;
    [X,Y] = meshgrid(xs, ys);
    setOccupancy(map, [X(:) Y(:)], occ);
end

function clearRectWorld(map, rect)
    fillRectWorld(map, rect, 0);
end

function slamMap = tryBuildMap(scansUsed, slamPoses, res, maxR, slamObj)
    slamMap = [];
    try
        slamMap = buildMap(scansUsed, slamPoses, res, maxR);
    catch
        try
            slamMap = buildMap(slamObj);
        catch
            slamMap = [];
        end
    end
end
