function [cable_force,num_vertex] = calculate_CFEL(force_ee,moment_ee,jaco,force_min,force_max,ij_indx)


x_vertex=zeros(120,2);
i2=0;
A=0;
sum1=0;
sum2=0;
% Particular solution of 'J'*F+w=0'
t_p=-jaco*inv(jaco'*jaco)*[force_ee;moment_ee];
% Null space of J'
n_s=null(jaco');
if (size(n_s,2)~=2)
   error('singular position')
end
% Aeq and Beq of the 2*8 linear equations.
% Aeq1=combntns([n_s(:,1);n_s(:,1)],2);
% Aeq2=combntns([n_s(:,2);n_s(:,2)],2);
% Beq0=combntns([force_min-t_p;force_max-t_p],2);

% combntns has been removed in MATLAB R2023b, use nchoosek instead
Aeq1=nchoosek([n_s(:,1);n_s(:,1)],2);
Aeq2=nchoosek([n_s(:,2);n_s(:,2)],2);
Beq0=nchoosek([force_min-t_p;force_max-t_p],2);

% solve the C16,2=120 linear equations
for i1=1:120
   Aeq=[(Aeq1(i1,:))' (Aeq2(i1,:))'];
   Beq=(Beq0(i1,:))';
% if the two line are parallel
   if (abs(det(Aeq))<1e-6)
      continue
   end
   x=inv(Aeq)*Beq;
% examine x_vertex meet the inequation constrain
% when compare two numbers by matlab, calculation tolerance must be considered
   if (((n_s*x)<=(force_max-t_p+1e-5))&((n_s*x)>=(force_min-t_p-1e-5)))
   i2=i2+1;
   x_vertex(i2,:)=x;
   end
end
% calculate the area of the polygon
% sort all the vertex

xs_vertex=sort_vertex(x_vertex(1:i2,:));
num_vertex=size(xs_vertex,1);
% increase one line; the last point is coinsidented with the first point
xs_vertex(i2+1,:)=xs_vertex(1,:);
for i3=1:i2
   A=xs_vertex(i3,1)*xs_vertex(i3+1,2)-xs_vertex(i3+1,1)*xs_vertex(i3,2)+A;
end
A=A/2;
% calculate the bary center
for i4=1:i2
   sum1=(xs_vertex(i4,1)+xs_vertex(i4+1,1))*...
      (xs_vertex(i4,1)*xs_vertex(i4+1,2)-xs_vertex(i4+1,1)*xs_vertex(i4,2))+sum1;
   sum2=(xs_vertex(i4,2)+xs_vertex(i4+1,2))*...
      (xs_vertex(i4,1)*xs_vertex(i4+1,2)-xs_vertex(i4+1,1)*xs_vertex(i4,2))+sum2;
end
b_c=[sum1/6/A;sum2/6/A];
% cable force distribution
cable_force=t_p+n_s*b_c;


N_C2 = inv(n_s(ij_indx,:));
n_star = n_s*N_C2;
delta_T = zeros(1,8);


tp_star = t_p - n_star*t_p(ij_indx);
% delta_T1 = zeros(1,8);
% delta_T2 = zeros(1,8);
% delta_T3 = zeros(1,8);
% delta_T4 = zeros(1,8);




% kk1 = 1;
% kk2 = 1;
% kk3 = 1;
% kk4 = 1;
L_matrix = [1 1 -1 -1;1 -1 1 -1;];

% T_k = zeros(8,1);
% for i = 1:8
%     T_k(i) = t_p(i)+n_star(i,:)*[]
% 
% end


for ii = 1:8

    for jj = 1:4
        L = L_matrix(:,jj);
        NL = (n_star(ii,:)*L);
        Tmax(jj) = (force_max-cable_force(ii))/NL;
        Tmin(jj) = (force_min-cable_force(ii))/NL;

        if abs(Tmax(jj)-0.66)<0.1 || abs(Tmin(jj)-0.66)<0.1
            nnn = 1;
        end


        % if jj == 1
        %     delta_T1(kk1) = max([Tmax,Tmin]);
        %     kk1 = kk1+1;
        % end
        % 
        % if jj == 2
        %     delta_T2(kk2) = max([Tmax,Tmin]);
        %     kk2 = kk2+1;
        % end
        % 
        % if jj == 3
        %     delta_T3(kk3) = max([Tmax,Tmin]);
        %     kk3 = kk3+1;
        % end
        % 
        % if jj == 4
        %     delta_T4(kk4) = max([Tmax,Tmin]);
        %     kk4 = kk4+1;
        % end

    end




    delta_T(ii) = max([Tmax,Tmin]);

    % end
end


min(delta_T)



end