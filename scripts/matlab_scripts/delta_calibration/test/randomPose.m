% Generates a set of random poses with values within range
function p = randomPoses(range, num)
    rng('shuffle','twister');
    a = -range;
    b = range;
    p = ((b-a).*rand(6,1) + a)';
    for n = 2:num
        % Generate random values in a range
        rng('shuffle','twister');
        a = -range;
        b = range;
        p = [p ;((b-a).*rand(6,1) + a)'];
    end