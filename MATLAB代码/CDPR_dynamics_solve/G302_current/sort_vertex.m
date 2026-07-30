function sort_p=sort_vertex(ini_p)
% this programme is used to give a counterclockwise order of the vertex of a 
% convex polyhedron. Initial points coordinates are saved in 'ini_p'
% examine the coordinates of the input points, if there is imag value, stop
if (~isreal(ini_p))
   error('at least one coordinate of the vertex of convex polyhedron is imag')
end
if (size(ini_p,1)<3)
   error('number of vertex is less than 3')
end
% x,y coorinate of every point
x=ini_p(:,1);
y=ini_p(:,2);
% sort
[ignore I] = sort(angle(complex(x-mean(x),y-mean(y))));
x_s = x(I);
y_s = y(I);
sort_p=[x_s y_s];
end