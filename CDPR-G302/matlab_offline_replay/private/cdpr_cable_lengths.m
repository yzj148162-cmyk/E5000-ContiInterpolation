function lengths = cdpr_cable_lengths(pose, cfg)
anchors = double(cfg.geometry.base_anchor_global_mm);
pointsByEnd = cfg.geometry.platform_attachment_local_mm_by_end;
if iscell(pointsByEnd)
    local = double(pointsByEnd{1});
else
    local = double(pointsByEnd);
    if ndims(local) == 3
        local = squeeze(local(1,:,:));
    end
end
if size(local,2) ~= 3 && size(local,1) == 3
    local = local.';
end
if size(anchors,2) ~= 3 && size(anchors,1) == 3
    anchors = anchors.';
end
R = rotz_local(pose(6)) * roty_local(pose(5)) * rotx_local(pose(4));
globalPoint = (R * local.').'+ pose(1:3);
lengths = zeros(1,8);
for i = 1:8
    lengths(i) = pulley_cable_length(globalPoint(i,:), anchors(i,:), ...
        double(cfg.geometry.pulley_radius_mm));
end
end

function lengthMm = pulley_cable_length(contact, anchor, radius)
direct = norm(anchor-contact);
if ~isfinite(direct) || radius <= 0
    lengthMm = direct;
    return;
end
scale = max(abs([contact anchor]));
upper = anchor(3) >= (scale > 100) * 999 + 1;
if upper
    theta = atan2(abs(anchor(1)-contact(1)), abs(anchor(2)-contact(2)));
    center = [anchor(1)-sign_nonzero(anchor(1))*radius*sin(theta), ...
              anchor(2)-sign_nonzero(anchor(2))*radius*cos(theta), anchor(3)];
else
    theta = atan2(abs(anchor(2)-contact(2)), abs(anchor(1)-contact(1)));
    center = [anchor(1)-sign_nonzero(anchor(1))*radius*cos(theta), ...
              anchor(2)-sign_nonzero(anchor(2))*radius*sin(theta), anchor(3)];
end
oa = contact-center; ob = anchor-center;
cosAob = max(-1,min(1,dot(oa,ob)/(norm(oa)*norm(ob))));
cosAoe = max(-1,min(1,radius/norm(oa)));
thetaBoe = 2*pi-acos(cosAob)-acos(cosAoe);
vertical = radius*cos(thetaBoe-pi/2);
horizontal = radius+radius*sin(thetaBoe-pi/2);
if upper
    tangent = [anchor(1)-sign_nonzero(anchor(1))*horizontal*sin(theta), ...
               anchor(2)-sign_nonzero(anchor(2))*horizontal*cos(theta), ...
               center(3)+vertical];
else
    tangent = [anchor(1)-sign_nonzero(anchor(1))*horizontal*cos(theta), ...
               anchor(2)-sign_nonzero(anchor(2))*horizontal*sin(theta), ...
               center(3)-vertical];
end
lengthMm = norm(contact-tangent)+radius*thetaBoe;
end

function s = sign_nonzero(x)
if x > 0, s = 1; else, s = -1; end
end
function R=rotx_local(a), R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)]; end
function R=roty_local(a), R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)]; end
function R=rotz_local(a), R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1]; end
