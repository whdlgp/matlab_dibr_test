function im_another_point = render_inverse_mapping(world_coord, ref_image, fx, fy, ox, oy)
    % Find pixel position if project to image plane with R|T applied points
    [height, width, ~] = size(ref_image);
    depth_expected = zeros(3, width*height);
    
    idx = 1:width*height;
    depth_expected(1, idx) = world_coord(1, idx)*fx./world_coord(3, idx) + ox;
    depth_expected(2, idx) = world_coord(2, idx)*fy./world_coord(3, idx) + oy;
    depth_expected(3, idx) = world_coord(3, idx);

    depth_expected = round(depth_expected); % round

    % Locate expected pixel postion to real image
    im_another_point = uint8(zeros(size(ref_image)));
    for idx = 1:width*height
        if mod(idx, width) == 0
            im1_rows = width;
            im1_cols = fix(idx / width);
        else
            im1_rows = mod(idx, width);
            im1_cols = fix(idx / width) + 1;
        end

        if depth_expected(1, idx) > 0 ...
           && depth_expected(2, idx) > 0 ...
           && depth_expected(1, idx) <= width...
           && depth_expected(2, idx) <= height
        im_another_point(im1_cols, im1_rows, :) = ref_image(depth_expected(2, idx), depth_expected(1, idx), :);
        end
    end
end

