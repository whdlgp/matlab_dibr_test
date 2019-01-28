function world_coord = d_pixel2world(depth, fx, fy, ox, oy)
    [height, width] = size(depth);
    world_coord = zeros(3, width*height);
    for x_im = 1:width
        for y_im = 1:height
            idx = (y_im - 1)*width + x_im;
            
            world_coord(1, idx) = (x_im - ox)*(depth(y_im, x_im)/fx);
            world_coord(2, idx) = (y_im - oy)*(depth(y_im, x_im)/fy);
            world_coord(3, idx) = depth(y_im, x_im);
        end
    end
end
