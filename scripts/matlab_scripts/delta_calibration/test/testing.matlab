function  = randomPose()
	% Generate random values in a range
	rng(0,'twister');
	a = 0;
	b = 1;
	r_axis = (b-a).*rand(3,1) + a

	a = -20;
	b = 20;
	translation = (b-a).*rand(3,1) + a