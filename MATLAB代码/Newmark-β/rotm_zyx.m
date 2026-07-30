function R = rotm_zyx(phi, theta, psi)
% Body-to-world rotation matrix for ZYX Euler angles.
cphi = cos(phi);
sphi = sin(phi);
ctheta = cos(theta);
stheta = sin(theta);
cpsi = cos(psi);
spsi = sin(psi);

R = [cpsi*ctheta, cpsi*stheta*sphi - spsi*cphi, cpsi*stheta*cphi + spsi*sphi;
     spsi*ctheta, spsi*stheta*sphi + cpsi*cphi, spsi*stheta*cphi - cpsi*sphi;
     -stheta,     ctheta*sphi,                      ctheta*cphi];
end
