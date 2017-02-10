function stripped = StripTranslation(tf, u)
  r_aa = tf(1:3);
  t = tf(4:6);
  tf
  u
  r_q = aa2quat(r_aa);
  r_v = [r_q(0), r_q(1), r_q(2)]
  r_part = dot(r_v', u')/norm(u) * u'
  r_m = quat2rotm(r_part);
  stripped = r_m' * quat2rotm(r_q);
  stripped = [r_aa stripped 0 0]
  stripped = [stripped t 0 0]
end