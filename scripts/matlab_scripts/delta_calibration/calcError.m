function e = calc_error(A1, A2)
    rot1 = aa2rotm(A1(1:3)');
    rot2 = aa2rotm(A2(1:3)');
    t1 = A1(4:6)';
    t2 = A2(4:6)';
    rotError = rot2 * inv(rot1);
    t1;
    t2;
    inv(rot1) * -t1;
    tError = rot2 * (inv(rot1) * - t1) + t2;
    
    aaError = rotm2aa(rotError);
    angleMagnitudeE = norm(aaError)
    translationMagnitudeE = norm(tError)
    e = [aaError tError']