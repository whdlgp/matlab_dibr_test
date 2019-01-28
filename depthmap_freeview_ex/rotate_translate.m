function [world_coord_rt, rot_mat, t_mat] = rotate_translate(world_coord, alpha, beta, gamma, tx, ty, tz)
    % Rotation and Translation matrix
    rot_x = [1 0 0;0 cos(alpha) -sin(alpha);0 sin(alpha) cos(alpha)];
    rot_y = [cos(beta) 0 sin(beta);0 1 0;-sin(beta) 0 cos(beta)];
    rot_z = [cos(gamma) -sin(gamma) 0;sin(gamma) cos(gamma) 0;0 0 1];
    rot_mat = rot_z*rot_y*rot_x;
    t_mat = [tx ty tz]';
    
    [coord_num, length] = size(world_coord);
    world_coord_rt = zeros(coord_num, length);
    for idx = 1:length
        world_coord_rt(:, idx) = rot_mat*(world_coord(:, idx) - t_mat);
    end
end

