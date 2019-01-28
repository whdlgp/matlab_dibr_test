function show_depth(depth)
    [height, width] = size(depth);
    valid = 0;
    for w = 1:width
        for h = 1:height
            if depth(h, w) > 10
                valid = valid+1;
            end
        end
    end
    
    vect = zeros(3, valid);
    count = 0;
    for w = 1:width
        for h = 1:height
            if depth(h, w) > 10
                count = count+1;
                vect(1, count) = w;
                vect(2, count) = h;
                vect(3, count) = depth(h, w);
            end
        end
    end
    scatter3(vect(1, :), vect(2, :), vect(3, :), '.');
end

