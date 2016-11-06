function m2 = M2FromQuat(r1)
  w = r1(1);
  x = r1(2);
  y = r1(3);
  z = r1(4);
  m2 = [w, -x, -y, -z; 
        x,  w, -z,  y;
        y,  z,  w, -x;
        z, -y,  x,  w];
  m2(isnan(m2)) = 0 ;