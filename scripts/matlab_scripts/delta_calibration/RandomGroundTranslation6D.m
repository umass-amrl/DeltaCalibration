function tf = RandomGroundTranslation6D(max_angle, max_translation)
	rotation = RandomRotationAA(max_angle)
	translation = RandomGroundTranslation(max_translation)
	tf = [rotation; translation]
    tf = [tf; 0; 0]
end