function tf = RandomZAxisRotate(max_angle, max_translation)
	rotation = RandomZRotationAA(max_angle)
	translation = RandomTranslation(max_translation)
	tf = [rotation; translation]
    tf = [tf; 0; 0]
end