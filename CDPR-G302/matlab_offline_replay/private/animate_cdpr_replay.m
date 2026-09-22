function animate_cdpr_replay(result, options)
%ANIMATE_CDPR_REPLAY Synchronized 3-D platform and time-history dashboard.
cfg = result.config;
pose = result.actualPoseMmRad;
valid = all(isfinite(pose),2);
index = find(valid);
index = index(1:options.FrameStride:end);
if isempty(index), return; end

anchors = normalize_points(cfg.geometry.base_anchor_global_mm);
pointsByEnd = cfg.geometry.platform_attachment_local_mm_by_end;
if iscell(pointsByEnd), attachmentsLocal = double(pointsByEnd{1});
else, attachmentsLocal = double(pointsByEnd); end
attachmentsLocal = normalize_points(attachmentsLocal);
body = build_platform_visual_geometry(cfg);

timeS = result.sample.timeS(:);
desiredPosition = result.sample.desiredPose(:,1:3);
actualPosition = pose(:,1:3);
if isfield(result.sample, 'platformForceN')
    platformForce = result.sample.platformForceN;
else
    platformForce = nan(numel(timeS),3);
end

fig = figure('Name','CDPR-G302 六维力交互离线回放', ...
    'Color','w','Position',[80 60 1580 900]);
layout = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');

%% Left: complete CDPR spatial view.
ax = nexttile(layout,1,[3 1]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,3);
xlabel(ax,'X / mm'); ylabel(ax,'Y / mm'); zlabel(ax,'Z / mm');
frameMin = double(cfg.workspace.frame_minimum_mm(:));
frameMax = double(cfg.workspace.frame_maximum_mm(:));
xlim(ax,[frameMin(1) frameMax(1)]);
ylim(ax,[frameMin(2) frameMax(2)]);
zlim(ax,[frameMin(3) frameMax(3)]);
plot_frame_box(ax,frameMin,frameMax);
plot3(ax,anchors(:,1),anchors(:,2),anchors(:,3),'ks', ...
    'MarkerFaceColor','k','DisplayName','机架出绳点');

cables = gobjects(8,1);
for i = 1:8
    cables(i) = plot3(ax,nan,nan,nan,'Color',[0.15 0.38 0.85], ...
        'LineWidth',1.1,'HandleVisibility','off');
end
bodyPatch = patch(ax,'Faces',body.faces,'Vertices',body.verticesLocalMm, ...
    'FaceColor',[0.10 0.65 0.82],'FaceAlpha',0.16, ...
    'EdgeColor','none','HandleVisibility','off');
bodyEdges = plot3(ax,nan,nan,nan,'Color',[0.02 0.25 0.48], ...
    'LineWidth',1.8,'DisplayName',sprintf('正二十面体（30边，边长%.1f mm）', ...
    body.meanEdgeLengthMm));
attachmentMarkers = plot3(ax,nan,nan,nan,'ro','MarkerFaceColor',[0.95 0.3 0.2], ...
    'MarkerSize',5,'DisplayName','8个绳索连接点');
actualCenter = plot3(ax,nan,nan,nan,'o','Color',[0.85 0.05 0.05], ...
    'MarkerFaceColor',[0.85 0.05 0.05],'DisplayName','实际质心');
desiredCenter = plot3(ax,nan,nan,nan,'x','Color',[0 0.55 0], ...
    'LineWidth',1.8,'MarkerSize',9,'DisplayName','期望质心');
actualPath = animatedline(ax,'Color',[0.85 0.05 0.05],'LineWidth',1.5, ...
    'HandleVisibility','off');
plot3(ax,desiredPosition(:,1),desiredPosition(:,2),desiredPosition(:,3), ...
    '--','Color',[0 0.55 0],'LineWidth',1.2,'HandleVisibility','off');
legend(ax,'Location','best');

%% Right-top: force entering the dynamics model in the global frame.
forceAx = nexttile(layout,2);
hold(forceAx,'on'); grid(forceAx,'on');
forceColors = [0.85 0.10 0.10; 0.10 0.55 0.18; 0.10 0.30 0.85];
forceLines = gobjects(3,1);
forceNames = {'F_x','F_y','F_z'};
for component = 1:3
    forceLines(component) = animatedline(forceAx, ...
        'Color',forceColors(component,:),'LineWidth',1.4, ...
        'DisplayName',forceNames{component});
end
forceCursor = xline(forceAx,timeS(index(1)),'k:','HandleVisibility','off');
xlim(forceAx,time_limits(timeS));
ylim(forceAx,padded_limits(platformForce,[-1 1]));
xlabel(forceAx,'时间 / s'); ylabel(forceAx,'力 / N');
title(forceAx,'进入动力学模型的全局系三维力');
legend(forceAx,'Location','best','Orientation','horizontal');

%% Right-bottom: desired and actual Cartesian position.
positionAx = nexttile(layout,4,[2 1]);
hold(positionAx,'on'); grid(positionAx,'on');
desiredLines = gobjects(3,1);
actualLines = gobjects(3,1);
axisNames = {'X','Y','Z'};
for component = 1:3
    desiredLines(component) = animatedline(positionAx, ...
        'Color',forceColors(component,:),'LineStyle','--','LineWidth',1.15, ...
        'DisplayName',[axisNames{component} '期望']);
    actualLines(component) = animatedline(positionAx, ...
        'Color',forceColors(component,:),'LineStyle','-','LineWidth',1.7, ...
        'DisplayName',[axisNames{component} '实际']);
end
positionCursor = xline(positionAx,timeS(index(1)),'k:','HandleVisibility','off');
xlim(positionAx,time_limits(timeS));
ylim(positionAx,padded_limits([desiredPosition; actualPosition],[-1 1]));
xlabel(positionAx,'时间 / s'); ylabel(positionAx,'位置 / mm');
title(positionAx,'期望位置与Trace正运动学实际位置');
legend(positionAx,'Location','best','NumColumns',3);

writer = [];
videoTemporaryFile = '';
videoFinalFile = '';
if options.WriteVideo
    % VideoWriter在Windows下仍受较短路径限制，而Qt日志目录通常很深。
    % 先写系统临时目录，关闭编码器后再移动到正式回放目录。
    videoFinalFile = char(fullfile(result.outputFolder, ...
        "virtual_platform_replay.mp4"));
    videoTemporaryFile = [tempname(tempdir) '.mp4'];
    writer = VideoWriter(videoTemporaryFile,'MPEG-4');
    writer.FrameRate = 25;
    open(writer);
end
cleanup = onCleanup(@() finalize_video_writer( ...
    writer,videoTemporaryFile,videoFinalFile));

previousK = 0;
for k = index(:).'
    if ~isgraphics(fig), break; end
    p = pose(k,:);
    R = rotz_local(p(6))*roty_local(p(5))*rotx_local(p(4));
    attachmentsGlobal = (R*attachmentsLocal.').'+p(1:3);
    verticesGlobal = (R*body.verticesLocalMm.').'+p(1:3);
    set(bodyPatch,'Vertices',verticesGlobal);
    [edgeX,edgeY,edgeZ] = edge_coordinates(verticesGlobal,body.edges);
    set(bodyEdges,'XData',edgeX,'YData',edgeY,'ZData',edgeZ);
    set(attachmentMarkers,'XData',attachmentsGlobal(:,1), ...
        'YData',attachmentsGlobal(:,2),'ZData',attachmentsGlobal(:,3));
    for i = 1:8
        set(cables(i),'XData',[anchors(i,1) attachmentsGlobal(i,1)], ...
            'YData',[anchors(i,2) attachmentsGlobal(i,2)], ...
            'ZData',[anchors(i,3) attachmentsGlobal(i,3)]);
    end
    set(actualCenter,'XData',p(1),'YData',p(2),'ZData',p(3));
    set(desiredCenter,'XData',desiredPosition(k,1), ...
        'YData',desiredPosition(k,2),'ZData',desiredPosition(k,3));
    addpoints(actualPath,p(1),p(2),p(3));

    history = (previousK+1):k;
    for component = 1:3
        addpoints(forceLines(component),timeS(history), ...
            platformForce(history,component));
        addpoints(desiredLines(component),timeS(history), ...
            desiredPosition(history,component));
        addpoints(actualLines(component),timeS(history), ...
            actualPosition(history,component));
    end
    forceCursor.Value = timeS(k);
    positionCursor.Value = timeS(k);
    previousK = k;

    title(ax,sprintf(['t=%.3f s，平移误差=%.3f mm\n' ...
        '%s；顶点/面/边=%d/%d/%d'],timeS(k), ...
        norm(p(1:3)-desiredPosition(k,:)),body.source, ...
        size(body.verticesLocalMm,1),size(body.faces,1),size(body.edges,1)));
    drawnow;
    if ~isempty(writer) && isgraphics(fig)
        writeVideo(writer,getframe(fig));
    end
end
clear cleanup;
end

function points = normalize_points(value)
points = double(value);
points = squeeze(points);
if size(points,2) ~= 3 && size(points,1) == 3
    points = points.';
end
end

function [x,y,z] = edge_coordinates(vertices,edges)
n = size(edges,1);
x = nan(3*n,1); y = x; z = x;
for edge = 1:n
    range = (3*edge-2):(3*edge-1);
    pair = edges(edge,:);
    x(range) = vertices(pair,1);
    y(range) = vertices(pair,2);
    z(range) = vertices(pair,3);
end
end

function plot_frame_box(ax,frameMin,frameMax)
v = [frameMin(1) frameMin(2) frameMin(3); ...
     frameMax(1) frameMin(2) frameMin(3); ...
     frameMax(1) frameMax(2) frameMin(3); ...
     frameMin(1) frameMax(2) frameMin(3); ...
     frameMin(1) frameMin(2) frameMax(3); ...
     frameMax(1) frameMin(2) frameMax(3); ...
     frameMax(1) frameMax(2) frameMax(3); ...
     frameMin(1) frameMax(2) frameMax(3)];
e = [1 2;2 3;3 4;4 1;5 6;6 7;7 8;8 5;1 5;2 6;3 7;4 8];
[x,y,z] = edge_coordinates(v,e);
plot3(ax,x,y,z,'Color',[0.35 0.35 0.35],'LineWidth',0.8, ...
    'HandleVisibility','off');
end

function limits = time_limits(timeS)
limits = [min(timeS) max(timeS)];
if limits(2) <= limits(1)
    limits = limits(1) + [0 1];
end
end

function limits = padded_limits(values,fallback)
finiteValues = values(isfinite(values));
if isempty(finiteValues)
    limits = fallback;
    return;
end
minimum = min(finiteValues);
maximum = max(finiteValues);
span = maximum - minimum;
if span <= max(1e-12,1e-9*max(abs([minimum maximum])))
    padding = max(1,0.05*max(1,abs(minimum)));
else
    padding = 0.08*span;
end
limits = [minimum-padding maximum+padding];
end

function finalize_video_writer(writer,temporaryFile,finalFile)
if isempty(writer), return; end
close(writer);
if ~isfile(temporaryFile)
    return;
end
[moved,message] = movefile(temporaryFile,finalFile,'f');
if ~moved
    error('CDPR:Replay:VideoMove', ...
        '视频已编码但无法移入结果目录：%s',message);
end
fprintf('视频：%s\n',finalFile);
end
function R=rotx_local(a), R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)]; end
function R=roty_local(a), R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)]; end
function R=rotz_local(a), R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1]; end
