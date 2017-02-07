function T = RandomTranslation(max_translation)
	direction = rand(3,1);
	direction = direction / norm(direction);
	T = direction;
end