function y = deltatTest()
%     [c0, c1] = translationTest();
%     % Calculate T and R from C1 and C0
%     [R, T] = calibrate(c0, c1);
%     checkResults(c0,c1, T, R);
    
    [c0, c1] = rotationTest();
    % Calculate T and R from C1 and C0
    [R, T] = calibrate(c0, c1);
    %checkResults(c0,c1, R, T);
    
%     [c0, c1] = randomTest();
%     % Calculate T and R from C1 and C0
%     [R, T] = calibrate(c0, c1);
%     checkResults(c0,c1, R, T);
    

function y = checkResults(c0,c1, R, T)
    % Apply T and R to C1
    c2 = quatRotate(R', c1);
    c2 = translate(c2, T');
    dY = c0 - c2;
    Err = norm(dY,'fro') % must use Frobenius norm
    
function [C0, C1] = rotationTest()
    % Two cameras rotated 180 degrees from each other
    % C1 = [ .9,  0,  .6,   3,  5,  7;
    %         0,  0,  .7,   0,  7,  0;
    %        .4, .7,   0,   0,  0,  4];
    % C0 = [  0,  -.9,  .6,   5,  -3,  7;
    %         0,  0,  .7,   7,  0,  0;
    %        .7, -.4,   0,   0,  0,  4]
    % C1 = [ 0,  0,  1.5,   0,  0,  0;
    %         1.5,  0,  0,   0,  0,  0;
    %        0, 0,   1.5,   0,  0,  0];
    % C0 = [  0,  0,  1.5,   2,  -2,  0;
    %         1.5,  0,  0,   0,  2,  -2;
    %        0, 0,   1.5,   0,  -2,  0];
    C1 = [ -2.0922309192,  -0.0609387646,  1.340652822,   1.21,  -.51,  2.08];
    C0 = [ -2.0734068523,  -0.2845852542,  -1.3416161985,   1.14,  .61,  -2.1];
    %%[R, T] = transformsFromPose(transform');
    %%c2 = quatRotate(R, C1);
    %c2 = translate(c2, T')
    %C0 = c2;
    %%C0 = quatRotate(R, C1)
    %%C0 = translate(C1, T');
 

function points = transformPoints(input, transforms)
    [R, T] = transformsFromPose(transform');
    points = quatRotate(R, input);
    points = translate(points, T');

function [C0, C1] = translationTest()
    points = [.7,0,0,0,0,0;
          .7,0,0,1,0,0;
          .7,0,0,0,1,0;
          .7,0,0,0,0,1;
          .7,0,0,1,0,1;
          .7,0,0,1,1,0;
          .7,0,0,0,1,1;
          .7,0,0,1,1,1]
    % Generate random C1 and the Transform
    randC1 = randomPose(20,50);
    % Translation along one axis
    transform = [0,0,.7,2,5,7];
    [R, T] = transformsFromPose(transform')
    randC2 = translate(randC1, T');
    C0 = randC1;
    C1 = randC2;

% Tests transform estimation using random poses and a random transform
function [randC0, randC1] = randomTest()
    % Generate random C1 and the Transform
    randC1 = randomPose(300,100);
    transform = randomPose(360, 1);
    % Convert the transform to a quaternion and apply the transform
    [R, T] = transformsFromPose(transform')
    randC2 = quatRotate(R, randC1);
    randC0 = translate(randC2, T');
    
% Generates a set of random poses with values within range
function p = randomPoses(range, num)
    rng('shuffle','twister');
    a = -range;
    b = range;
    p = ((b-a).*rand(6,1) + a)';
    for n = 2:num
        % Generate random values in a range
        rng('shuffle','twister');
        a = -range;
        b = range;
        p = [p ;((b-a).*rand(6,1) + a)'];
    end

% Generates a trajectory containing a number of points transformed by the given transform
function P = posesFromTransform(num, transform)
    [R,T] = transformsFromPose(transform');
    P = [];
    p0 = [0, .707, 0, 0, 0, 0];
    for n = 1:num
        p0 = translate(p0, T');
        p0 = quatRotate(R, p0);
        
        
        P = [P ; p0];
    end

% Calculates transforms from pose
function [r,t] = transformsFromPose(pose)
    rot = pose(1:3);
    t = pose(4:6);
    
    % Form the axis angle representation
    angle = norm(rot);
    axis = rot/angle;
    axAngle = [axis;angle];
    % Convert axis angle to quaternion
    r = axAngle2Quat(axAngle');

function q = axAngle2Quat(axAngle);
  angle = axAngle(4);
  qx = axAngle(1) * sin(angle/2);
  qy = axAngle(2) * sin(angle/2);
  qz = axAngle(3) * sin(angle/2);
  qw = cos(angle/2);
  q = [qw, qx, qy, qz];