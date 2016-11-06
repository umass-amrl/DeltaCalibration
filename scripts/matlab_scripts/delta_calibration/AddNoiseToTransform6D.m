function tf_noisy = AddNoiseToTransform6D(tf, noise_angular, noise_translation)
  aa = tf(1:3);
  aa_error = RandomRotationAA(noise_angular); 
  aa_noisy = rotm2aa(aa2rotm(aa) * aa2rotm(aa_error));

  t = tf(4:6);
  t_error = RandomTranslation(noise_translation);
  t_noisy = t + t_error;

  tf_noisy = [aa_noisy'; t_noisy];
end