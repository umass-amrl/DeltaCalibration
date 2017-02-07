function R = RandomZRotationAA(max_angle)
	axis = [0; 0; 1];
	axis = axis / norm(axis);
	angle = (2.0 * rand(1,1) - 1.0) * max_angle;
	R = axis * angle;
end