function  [ideal_cf,ideal_cf_exitflag]= pseudo_inverse(force_ee, moment_ee, jaco, force_min, force_max)

ideal_cf_exitflag = 1;
ideal_cf = -jaco*inv(jaco'*jaco)*[force_ee;moment_ee];


for i = 1:8

   if ideal_cf(i) > force_max || ideal_cf(i) < force_min
       ideal_cf_exitflag = -1;
   end
end




end