function T = RandomTranslation(max_translation)
	direction = rand(3,1);
	direction = direction / norm(direction);
	magnitude = rand(1,1) * max_translation;
	T = magnitude * direction;
end