function R = quat2rotm( Qrotation )
% quat2rotm: get a 3x3 rotation matrix
% R = quat2rotm( Qrotation )
% IN: 
%     Qrotation - quaternion describing rotation in the order [w, x, y, z]
% 
% OUT:
%     R - rotation matrix 
%     
% VERSION: 03.03.2012


w = Qrotation( 4 );
x = Qrotation( 1 );
y = Qrotation( 2 );
z = Qrotation( 3 );

Rxx = 1 - 2*(y^2 + z^2);
Rxy = 2*(x*y - z*w);
Rxz = 2*(x*z + y*w);

Ryx = 2*(x*y + z*w);
Ryy = 1 - 2*(x^2 + z^2);
Ryz = 2*(y*z - x*w );

Rzx = 2*(x*z - y*w );
Rzy = 2*(y*z + x*w );
Rzz = 1 - 2 *(x^2 + y^2);

R = [ 
    Rxx,    Rxy,    Rxz;
    Ryx,    Ryy,    Ryz;
    Rzx,    Rzy,    Rzz];