function [thetax,thetay,thetaz] = rotm2eulerangles( R )
    thetax = atan2(R(3,2), R(3,3));
    thetay = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    thetaz = atan2(R(2,1), R(1,1));