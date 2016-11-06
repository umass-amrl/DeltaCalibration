function poses = calibrate_files(fileX, fileY)
  c0 = dlmread(fileX, '\t');
  c1 = dlmread(fileY, '\t');
  c0 = c0(:, 1:6);
  c1 = c1(:, 1:6);