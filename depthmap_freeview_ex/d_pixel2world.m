function world_coord = d_pixel2world(depth, fx, fy, ox, oy)
    [height, width] = size(depth);
    world_coord = zeros(3, width*height);
    
    x_im = 1:width;
    y_im = 1:height;
    [X_IM, Y_IM] = meshgrid(x_im, y_im);
    X_IM = reshape(X_IM, [1, width*height]);
    Y_IM = reshape(Y_IM, [1, width*height]);
    idx = (Y_IM - 1).*width + X_IM;
    depth_reshape = reshape(depth(y_im, x_im), [1, width*height]);
            
    world_coord(1, idx) = (X_IM - ox).*(depth_reshape/fx);
    world_coord(2, idx) = (Y_IM - oy).*(depth_reshape/fy);
    world_coord(3, idx) = depth_reshape;
end
