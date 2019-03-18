function world_coord_rt = rotate_translate_reverse(world_coord, rot_mat, t_mat)
    [coord_num, length] = size(world_coord);
    world_coord_rt = zeros(coord_num, length);
    idx = 1:length;
    world_coord_rt(:, idx) = rot_mat\world_coord(:, idx) + t_mat;
end

