function geometry = build_platform_visual_geometry(cfg)
%BUILD_PLATFORM_VISUAL_GEOMETRY Build the complete moving-platform mesh.
% New replay JSON files may provide all body vertices explicitly.  Existing
% G302 files only contain eight cable attachment vertices of a regular
% icosahedron, so the four non-cable vertices are reconstructed here.

if isfield(cfg.geometry, 'platform_body_vertices_local_mm')
    vertices = normalize_points(cfg.geometry.platform_body_vertices_local_mm);
    source = "JSON完整平台顶点";
else
    pointsByEnd = cfg.geometry.platform_attachment_local_mm_by_end;
    if iscell(pointsByEnd), attachments = double(pointsByEnd{1});
    else, attachments = double(pointsByEnd); end
    attachments = normalize_points(attachments);
    [vertices, ok] = infer_g302_regular_icosahedron(attachments);
    if ok
        source = "由8个绳索连接点恢复的正二十面体";
    else
        vertices = unique(attachments, 'rows', 'stable');
        source = "连接点凸包（正二十面体恢复失败）";
        warning('CDPR:Replay:PlatformGeometry', ...
            '无法从当前连接点恢复正二十面体，动画退回连接点凸包。');
    end
end

if size(vertices,1) < 4
    error('CDPR:Replay:PlatformGeometry', '动平台顶点不足，无法建立三维外形。');
end
faces = convhulln(vertices);
edges = unique(sort([faces(:,[1 2]); faces(:,[2 3]); faces(:,[3 1])], 2), ...
    'rows');
edgeLength = vecnorm(vertices(edges(:,1),:) - vertices(edges(:,2),:), 2, 2);

geometry = struct;
geometry.verticesLocalMm = vertices;
geometry.faces = faces;
geometry.edges = edges;
geometry.edgeLengthMm = edgeLength;
geometry.meanEdgeLengthMm = mean(edgeLength);
geometry.source = source;
geometry.isCompleteIcosahedron = size(vertices,1) == 12 && ...
    size(faces,1) == 20 && size(edges,1) == 30 && ...
    max(abs(edgeLength - mean(edgeLength))) <= 0.02 * mean(edgeLength);
end

function points = normalize_points(value)
points = double(value);
points = squeeze(points);
if size(points,2) ~= 3 && size(points,1) == 3
    points = points.';
end
if size(points,2) ~= 3 || any(~isfinite(points), 'all')
    error('CDPR:Replay:PlatformGeometry', ...
        '动平台局部顶点必须是有限的N×3数组。');
end
end

function [vertices, ok] = infer_g302_regular_icosahedron(attachments)
vertices = attachments;
ok = false;
if size(attachments,1) ~= 8
    return;
end

center = mean(attachments, 1);
q = attachments - center;
zValues = sort(q(:,3));
lower = q(:,3) <= mean(zValues(4:5));
upper = ~lower;
if nnz(lower) ~= 4 || nnz(upper) ~= 4
    return;
end
zLower = mean(q(lower,3));
zUpper = mean(q(upper,3));
ringHalfHeight = 0.5 * (zUpper - zLower);
ringCenterZ = 0.5 * (zUpper + zLower);
ringRadius = mean(hypot(q(:,1), q(:,2)));
scale = max(vecnorm(q,2,2));
if scale <= eps || ringHalfHeight <= 0 || ...
        abs(ringCenterZ) > 0.02 * scale || ...
        abs(ringRadius / (2 * ringHalfHeight) - 1) > 0.03
    return;
end

[missingLower, lowerOk] = missing_ring_vertex(q(lower,:), zLower, ringRadius);
[missingUpper, upperOk] = missing_ring_vertex(q(upper,:), zUpper, ringRadius);
if ~lowerOk || ~upperOk
    return;
end

% 对以一对顶点为Z轴的正二十面体，五边形环高度为R/sqrt(5)。
poleRadius = ringHalfHeight * sqrt(5);
bottomPole = [0 0 ringCenterZ - poleRadius];
topPole = [0 0 ringCenterZ + poleRadius];
vertices = [q; missingLower; missingUpper; bottomPole; topPole] + center;

faces = convhulln(vertices);
edges = unique(sort([faces(:,[1 2]); faces(:,[2 3]); faces(:,[3 1])], 2), ...
    'rows');
lengths = vecnorm(vertices(edges(:,1),:) - vertices(edges(:,2),:), 2, 2);
ok = size(faces,1) == 20 && size(edges,1) == 30 && ...
    max(abs(lengths - mean(lengths))) <= 0.02 * mean(lengths);
end

function [missing, ok] = missing_ring_vertex(ring, z, radius)
angles = sort(mod(atan2(ring(:,2), ring(:,1)), 2*pi));
gaps = diff([angles; angles(1) + 2*pi]);
[largestGap, index] = max(gaps);
otherGaps = gaps;
otherGaps(index) = [];
regularGap = 2*pi/5;
ok = abs(largestGap - 2*regularGap) < deg2rad(3) && ...
    max(abs(otherGaps - regularGap)) < deg2rad(3);
if ~ok
    missing = [nan nan nan];
    return;
end
angle = angles(index) + largestGap/2;
missing = [radius*cos(angle), radius*sin(angle), z];
end
