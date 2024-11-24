function [ x_rotated,y_rotated ] = Rotatepoints( x,y,theta,cor )
%accepts set of data points x & y to perform rotation with angle theta
%Counter CW about center of rotation cor

datamatrix = [x;y]; % create a matrix of the points to rotate

% choose a point which will be the center of rotation
x_center = cor(1);
y_center = cor(2);

% create a matrix which will be used later in calculations
center = repmat([x_center; y_center], 1, length(x));

% define the counter clockwise angle of rotation in rad       
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

% do the rotation...
s = datamatrix - center;     % shift points in the plane so that the center of rotation is at the origin
so = R*s; % apply the rotation about the origin
datamatrixo = so + center;   % shift again so the origin goes back to the desired center of rotation

% pick out the vectors of rotated x- and y-data
x_rotated = datamatrixo(1,:);
y_rotated = datamatrixo(2,:);


end

