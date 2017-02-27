function [t_err, r_err] = genBrassCal(N,angle,noise)

% clc;
t_range = 3;
r_range = 3;
for x=0:t_range
    for y=0:t_range
        for z=0:t_range
            for xr=0:r_range
                for yr=0:r_range
                    for zr=0:r_range
                        [xr yr zr x y z]
                        stringer = [num2str(xr,'%d') '_'  num2str(yr,'%d')  '_'  num2str(zr,'%d') '_'  num2str(x,'%d')  '_'  num2str(y,'%d')  '_'  num2str(z,'%d')]
                        xrR = (xr * 10) * (pi / 180);
                        yrR = (yr * 10) * (pi / 180);
                        zrR = (zr * 10) * (pi / 180);
                        
                        A = [xrR yrR zrR ((x*5) / 100) ((y*5) / 100) ((z*5) / 100) 0 0];
%N = 10;
max_delta_angle = angle / 180 * pi;
max_delta_translation = 0.2;
noise_angular = (1.0 *noise) / 180 * pi;
noise_translation = 0.01 * noise;

A1 = [];
A1I = [];
A2 = [];
A2I = [];
A1T = [];
A2T = [];
U1t = [0 0 0];
U2t = [0 0 0];
U1r = [0 0 0];
U2r = [0 0 0];
small_empty = [0 0 0];
for i=1:N
  Ar = RandomZAxisRotate(max_delta_angle, 0)';
%   A1 = [A1; RandomZAxisRotate(max_delta_angle, 0)'];
  At = RandomGroundTranslation6D(0, max_delta_translation)';
  A1 = [A1; Ar];
  A1T = [A1T; At];
  %A1(end,:) = AddNoiseToTransform6D(A1(end,:)', noise_angular, noise_translation)';
  A2 = [A2; A1toA2(A', Ar')];
  A2T = [A2T; A1toA2(A', At')];
  U1t = [U1t; small_empty];
  U2t = [U2t; small_empty];
  U1r = [U1r; small_empty];
  U2r = [U2r; small_empty];
  %A2(end,:) = AddNoiseToTransform6D(A2(end,:)', noise_angular, noise_translation)';
end

%% ===========================================
% Test Calibration Math with generated data.
C0 = [A1 A2]
C1 = [A1T A2T];
Ut = [U1t U2t];
Ur = [U1r U2r];
size(C0);

dlmwrite([stringer '_rot' '.txt'], C0, ' ');
dlmwrite([stringer '_trans' '.txt'], C1, ' ');
dlmwrite([stringer '_rot' '.txt'], C0, ' ');
dlmwrite([stringer '_uncT' '.txt'], Ut, ' ');
dlmwrite([stringer '_uncR' '.txt'], Ur, ' ');
%Add file writing ( need two files per)
                    end
                end
            end
        end
    end
end