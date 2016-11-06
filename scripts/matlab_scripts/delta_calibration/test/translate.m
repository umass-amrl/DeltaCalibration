function p = translate(pose, trans)    
    pose(:, 4:6) = bsxfun(@plus,pose(:, 4:6),trans);
    p = pose;