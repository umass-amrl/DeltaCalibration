function tf = RandomTransform6D(max_angle, max_translation)
	rotation = RandomRotationAA(max_angle)
	translation = RandomTranslation(max_translation)
	tf = [rotation; translation]
    tf = [tf; 0; 0]
end