%% --- Kartparametre ---
res     = 0.1;         % rutenett-oppløsning [m/celle]
nx      = 120;         % antall kolonner  -> ~ 12.0 m
ny      = 80;          % antall rader     -> ~  8.0 m
[occ,res] = makeWarehouseMap(nx, ny, res, ...
                             'shelfThickness', 0.8, ...  % m
                             'aisleWidth',     2.0, ...  % m (ganger mellom hyller)
                             'crossAisleY',    2.5);     % m (tverrgang nederst)

worldW = nx*res;  worldH = ny*res;

%% --- Start/mål (meter) ---
start_m = [0.8, 0.8];                   % nær dokk/åpen flate
goal_m  = [11.0, 6.8];                  % inni en gang ved hyller

% Konverter til celler og sikre at de er i fri sone
start_rc = world2cell(start_m, res);
goal_rc  = world2cell(goal_m,  res);
start_rc = snapToFree(occ, start_rc);
goal_rc  = snapToFree(occ,  goal_rc);

%% --- A* på rutenett ---
path_rc = astar_grid(occ, start_rc, goal_rc);   % [r c] liste
if isempty(path_rc)
    error('Fant ingen farbar rute i kartet. Juster kart/Start/Mål.');
end

% Konverter rute til meter og forenkle
path_xy = cell2world(path_rc, res);      % [x y] i meter
path_xy = simplifyPath(path_xy);         % fjern overflødige knekk
[path_xy, s_cum] = densifyPath(path_xy, 0.05); % jevn sampling for kontroll

%% --- Visualisering av kart + planlagt rute ---
fig = figure('Name','Lagerkart og rute','NumberTitle','off');
ax  = axes(fig); hold(ax,'on'); axis(ax,'equal')
drawWarehouseMap(ax, occ, res);
plot(ax, path_xy(:,1), path_xy(:,2), 'k--', 'LineWidth', 1.2, 'DisplayName','Planlagt rute');
plot(ax, start_m(1), start_m(2), 'go', 'MarkerFaceColor','g', 'DisplayName','Start');
plot(ax, goal_m(1),  goal_m(2),  'rx', 'LineWidth', 2, 'DisplayName','Mål');
legend(ax, 'Location','northeastoutside'); xlim([0 worldW]); ylim([0 worldH]);
title(ax, 'Lagerkart (fri = hvit, hylle = grå)');

%% --- Ackermann / pure pursuit simulering ---
L   = 1.0;         % akselavstand [m]
v   = 0.6;         % m/s konstant fart
Ld  = 0.8;         % lookahead [m]
dt  = 0.05;        % tidssteg [s]
delta_max = deg2rad(40);  % maks styrevinkel [rad]

% Init pos og heading mot første del av ruta
x = start_m(1);  y = start_m(2);
if size(path_xy,1) >= 2
    th = atan2(path_xy(2,2)-path_xy(1,2), path_xy(2,1)-path_xy(1,1));
else
    th = 0;
end

% Tegn truck som trekantpatch
[carX, carY] = carFootprint(x, y, th, L, 0.6);
hpCar = patch(ax, carX, carY, [0 0.447 0.741], 'EdgeColor','none', 'FaceAlpha',0.9, 'DisplayName','Truck');

% Trail
hpTrail = plot(ax, x, y, 'b-', 'LineWidth', 1.5, 'DisplayName','Kjørt bane');

% Robust animasjon (stopp på 'q' / lukk)
setappdata(fig,'alive', true);
set(fig,'KeyPressFcn',     @keyHandler);
set(fig,'CloseRequestFcn', @closeHandler);

% Precompute rutegeometri
route = struct('xy', path_xy, 's', s_cum);

% Simuler
t = 0; trailX = x; trailY = y;
while getappdata(fig,'alive')
    % Finn lookahead-punkt Ld meter foran nåværende pos langs ruten
    [s_here, p_here]     = closestPointOnPath([x y], route);
    [p_target, atEnd]    = pointAtArcLength(route, s_here + Ld);
    if atEnd && hypot(goal_m(1)-x, goal_m(2)-y) < 0.2
        % Mål nådd
        break;
    end

    % Transformér lookahead til bilens ramme
    R = [cos(th) -sin(th); sin(th) cos(th)];
    local = (p_target - [x y]) * R;   % verdens->lokalt (rad transponert)
    lx = local(1); ly = local(2);
    if lx < 1e-3
        kappa = 0;    % ikke styr når målpunkt er rett "over skulderen"
    else
        kappa = 2*ly/(Ld^2 + eps);   % pure pursuit kurvatur
    end
    delta = atan(L * kappa);
    delta = max(min(delta, delta_max), -delta_max);

    % Integrér sykkelmodellen
    x  = x  + v*cos(th)*dt;
    y  = y  + v*sin(th)*dt;
    th = th + (v/L)*tan(delta)*dt;
    t  = t  + dt;

    % Oppdater tegning
    [carX, carY] = carFootprint(x, y, th, L, 0.6);
    if ishandle(hpCar)
        set(hpCar, 'XData', carX, 'YData', carY);
    end
    trailX(end+1) = x; %#ok<SAGROW>
    trailY(end+1) = y; %#ok<SAGROW>
    if ishandle(hpTrail)
        set(hpTrail, 'XData', trailX, 'YData', trailY);
    end
    drawnow limitrate
    pause(dt);
end

if ishandle(fig), setappdata(fig,'alive',false); end
fprintf('Sim ferdig. Tid: %.1f s, kjørt distanse: ~%.1f m\n', t, arcLength([trailX(:) trailY(:)]));

%% =================== Lokale funksjoner ===================

function [occ,res] = makeWarehouseMap(nx, ny, res, varargin)
% Lager et stilisert lagerkart med hyllerekker.
% occ(r,c)=1: blokkert (hylle/vegg), 0: fri.
    p = inputParser;
    addParameter(p,'shelfThickness',0.8);   % m
    addParameter(p,'aisleWidth',    2.0);   % m
    addParameter(p,'crossAisleY',   1.5);   % m (tverrgang fra venstre->høyre)
    parse(p,varargin{:});
    th = p.Results.shelfThickness;
    aw = p.Results.aisleWidth;
    cy = p.Results.crossAisleY;

    occ = zeros(ny, nx, 'uint8');

    % Ytre vegger
    occ(1,:)   = 1; occ(end,:)=1;
    occ(:,1)   = 1; occ(:,end)=1;

    % Hyllerekker (horisontale blokker) fra x=1.0 m til x=(nx-1)*res - 1.0 m
    x0 = 1.0;  x1 = nx*res - 1.0;
    shelfCellsX = max(2, round(x0/res)) : min(nx-1, round(x1/res));

    % Plasser rader med (hylle tykkelse th) separert av ganger (aw)
    y = cy + aw;       % start litt over tverrgangen
    while y + th <= (ny*res - 1.0)
        r1 = max(2, round( y       /res));
        r2 = min(ny-1, round((y+th)/res));
        occ(r1:r2, shelfCellsX) = 1;
        y = y + th + aw;
    end

    % Tverrgang (sørg for åpning på bunn)
    r_cross = min(ny-1, max(2, round(cy/res)));
    occ(1:r_cross, :) = 0;   % åpen stripe nederst

    % Små søyler/paller som hindringer (valgfritt pynt)
    dots = [3  20;  3  70;  6  35;  6  95];  % (y, x) i meter
    for i=1:size(dots,1)
        rr = round(dots(i,1)/res);
        cc = round(dots(i,2)/res);
        rwin = max(2,rr-1):min(ny-1,rr+1);
        cwin = max(2,cc-1):min(nx-1,cc+1);
        occ(rwin, cwin) = 1;
    end
end

function drawWarehouseMap(ax, occ, res)
% Tegn kartet pent i meter
    imagesc(ax, [0 size(occ,2)*res], [0 size(occ,1)*res], occ);
    set(ax,'YDir','normal');
    colormap(ax, [1 1 1; 0.75 0.75 0.75]);  % fri=hvitt, hylle=lys grå
    grid(ax,'on'); ax.GridAlpha = 0.15;
    ax.XMinorGrid = 'on'; ax.YMinorGrid = 'on';
    ax.MinorGridAlpha = 0.05;
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
end

function rc = world2cell(xy, res)
    rc = [round(xy(2)/res + 0.5), round(xy(1)/res + 0.5)]; % [r c]
end
function xy = cell2world(rc, res)
    xy = [(rc(:,2)-0.5)*res, (rc(:,1)-0.5)*res];
end
function rc = snapToFree(occ, rc)
    % Flytt til nærmeste frie celle om nødvendig
    if occ(rc(1), rc(2))==0, return; end
    [R,C] = find(occ==0);
    d2 = (R-rc(1)).^2 + (C-rc(2)).^2;
    [~,i] = min(d2);
    rc = [R(i), C(i)];
end

function path = astar_grid(occ, startRC, goalRC)
% A* på 8-nabo rutenett. Occ=1 blokkert.
    ny = size(occ,1); nx = size(occ,2);
    start = sub2ind([ny nx], startRC(1), startRC(2));
    goal  = sub2ind([ny nx], goalRC(1),  goalRC(2));

    % Init
    g = inf(ny*nx,1); f = inf(ny*nx,1); came = zeros(ny*nx,1,'uint32');
    g(start)=0; f(start)=heuristic(start,goal,ny,nx);

    open = false(ny*nx,1); open(start)=true;

    while any(open)
        % Finn node i open med lavest f
        openIdx = find(open);
        [~,k] = min(f(openIdx));
        current = openIdx(k);
        open(current) = false;

        if current == goal, break; end

        [r,c] = ind2sub([ny nx], current);
        N = neighbors8(r,c,ny,nx);
        for j=1:size(N,1)
            rr = N(j,1); cc=N(j,2);
            nid = sub2ind([ny nx], rr, cc);
            if occ(rr,cc)==1, continue; end
            step = hypot(rr-r, cc-c);
            tg = g(current) + step;
            if tg < g(nid)
                came(nid) = current;
                g(nid)    = tg;
                f(nid)    = tg + heuristic(nid,goal,ny,nx);
                open(nid) = true;
            end
        end
    end

    % Rekonstruer
    if came(goal)==0
        path = [];
    else
        path = reconstructPath(came, goal, [ny nx]);
    end
end

function h = heuristic(a,b,ny,nx)
    [ar,ac] = ind2sub([ny nx], a);
    [br,bc] = ind2sub([ny nx], b);
    h = hypot(double(ar-br), double(ac-bc));
end

function N = neighbors8(r,c,ny,nx)
    rr = (r-1):(r+1); cc=(c-1):(c+1);
    rr = rr(rr>=1 & rr<=ny);
    cc = cc(cc>=1 & cc<=nx);
    [C,R] = meshgrid(cc,rr);
    R = R(:); C=C(:);
    idx = ~(R==r & C==c);
    N = [R(idx), C(idx)];
end

function path = reconstructPath(came, goal, sz)
    idx = goal; path_idx = idx;
    while came(idx)~=0
        idx = came(idx);
        path_idx(end+1) = idx;
    end
    path_idx = fliplr(path_idx);
    [r,c] = ind2sub(sz, path_idx);
    path = [r(:), c(:)];
end

function P = simplifyPath(P)
% Fjern punkter som ligger på samme rette linje (collinear)
    if size(P,1)<=2, return; end
    keep = true(size(P,1),1);
    for i=2:size(P,1)-1
        v1 = P(i,:)   - P(i-1,:);
        v2 = P(i+1,:) - P(i,:);
        if norm(v1)<1e-12 || norm(v2)<1e-12
            continue
        end
        ang = abs(atan2(v1(1)*v2(2)-v1(2)*v2(1), dot(v1,v2))); % vinkel mellom
        if ang < deg2rad(2)   % nesten rettlinjet
            keep(i) = false;
        end
    end
    P = P(keep,:);
end

function [P, s] = densifyPath(P, ds)
% Jevn sampling langs polyline hvert ds meter
    if size(P,1)<2, s=0; return; end
    seg = sqrt(sum(diff(P).^2,2));
    cum = [0; cumsum(seg)];
    L   = cum(end);
    s   = (0:ds:L).';
    Pn  = zeros(numel(s),2);
    k=1;
    for i=1:numel(s)
        si = s(i);
        while si>cum(k+1) && k < numel(seg)
            k=k+1;
        end
        t = (si - cum(k)) / max(seg(k),eps);
        Pn(i,:) = (1-t)*P(k,:) + t*P(k+1,:);
    end
    P = Pn;
end

function [s_here, p_here] = closestPointOnPath(p, route)
% Finn nærmeste punkt på ruta (arc length s + punkt)
    P = route.xy;  S = route.s;
    dmin = inf; p_here = P(1,:); s_here = 0;
    for k=1:size(P,1)-1
        A = P(k,:); B=P(k+1,:);
        AB = B - A; t = dot(p-A, AB) / max(dot(AB,AB), eps);
        t = max(0,min(1,t));
        proj = A + t*AB;
        d = norm(p - proj);
        if d<dmin
            dmin = d;
            p_here = proj;
            s_here = S(k) + t*norm(AB);
        end
    end
end

function [p, atEnd] = pointAtArcLength(route, s_query)
% Punkt på ruten gitt arc length s_query (klamrer til slutt)
    S = route.s;  P = route.xy;
    atEnd = false;
    if s_query >= S(end)
        p = P(end,:); atEnd = true; return;
    end
    k = find(S <= s_query, 1, 'last');
    if k == numel(S), p = P(end,:); atEnd=true; return; end
    ds = S(k+1) - S(k);
    t  = (s_query - S(k)) / max(ds, eps);
    p  = (1-t)*P(k,:) + t*P(k+1,:);
end

function L = arcLength(P)
    L = sum(sqrt(sum(diff(P).^2,2)));
end

function [X,Y] = carFootprint(x,y,th,L, W)
% Trekantskisse av bil (spiss i front)
    Lf = 0.5*L; Lb = -0.5*L; W2 = 0.5*W;
    poly = [ Lf   Lb   Lb;
              0   W2  -W2 ];
    R = [cos(th) -sin(th); sin(th) cos(th)];
    world = R*poly + [x; y];
    X = world(1,:); Y = world(2,:);
end

function keyHandler(src, evt)
    fig = ancestor(src,'figure');
    if strcmp(evt.Key,'q')
        setappdata(fig,'alive',false);
    end
end

function closeHandler(src, ~)
    if ishandle(src), setappdata(src,'alive',false); end
    delete(src);
end