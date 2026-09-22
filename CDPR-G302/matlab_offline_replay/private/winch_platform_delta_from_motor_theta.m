function deltaMm = winch_platform_delta_from_motor_theta(axisCfg, thetaRad)
fallback = abs(double(axisCfg.fallback_motor_rad_per_mm));
winch = axisCfg.winch;
enabled = logical(winch.enabled) && double(winch.radius_mm) > eps && ...
    isfinite(double(winch.pitch_mm_per_rev)) && ...
    isfinite(double(winch.projection_mm));
if ~enabled
    deltaMm = thetaRad / fallback;
    return;
end

radius = double(winch.radius_mm);
pitch = double(winch.pitch_mm_per_rev);
projection = double(winch.projection_mm);
lengthPerRev = hypot(2*pi*radius, pitch);
lengthPerRad = lengthPerRev / (2*pi);
takeup = thetaRad * lengthPerRad;
ratio = pitch / lengthPerRev;
initialOffset = 0.0;
if logical(winch.initial_axial_offset_valid)
    initialOffset = double(winch.initial_axial_offset_mm);
end
deltaMm = takeup + hypot(projection, initialOffset + ratio*takeup) ...
    - hypot(projection, initialOffset);
end
