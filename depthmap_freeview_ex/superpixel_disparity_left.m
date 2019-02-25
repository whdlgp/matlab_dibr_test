function [disp_map, label, label_num, centroids, centroids_est, valid_set] = superpixel_disparity_left(im_left, im_right)
    [label, label_num] = superpixels(im_left, 1000);

    props = regionprops(label);
    centroids = cat(1, props.Centroid);
    boundingboxs = round(cat(1, props.BoundingBox));

    centroids_est = [centroids, zeros(label_num, 2)];

    for i = 1:label_num
        window = double(imcrop(im_left, boundingboxs(i, :)));

        [win_h, win_w, ~] = size(window);
        [~, width, ~] = size(im_right);

        epipole = boundingboxs(i, 2);
        simil = zeros(1, width - win_w + 1);

        for cols = 1 : (width - win_w + 1)
            tmp = double(imcrop(im_right, [cols-0.5, epipole-0.5, win_w-1, win_h-1]));

            simil(cols) = sum(sum(sum(((tmp - mean2(tmp)).*(window - mean2(window)))/(std2(tmp)*std2(window)))))/numel(window);
        end

        [match_score, match_idx] = max(simil);

        disparity = round(boundingboxs(i, 1)) - match_idx;
        centroids_est(i, 1) = centroids_est(i, 1) - disparity;
        centroids_est(i, 3) = match_score;
        centroids_est(i, 4) = disparity;
    end                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          

    valid_idx = find((centroids_est(:, 3) > 0.707) & (centroids_est(:, 4) > 0));
    valid_num = numel(valid_idx);
    valid_set = zeros(valid_num, 5);
    valid_set(:, 1) = centroids(valid_idx, 1);
    valid_set(:, 2) = centroids(valid_idx, 2);
    valid_set(:, 3) = centroids_est(valid_idx, 1);
    valid_set(:, 4) = centroids_est(valid_idx, 2);
    valid_set(:, 5) = centroids_est(valid_idx, 4);

    [im_height, im_width, ~] = size(im_left);
    disp_map = zeros(im_height, im_width);
    for i = 1:valid_num
        cent_x = round(valid_set(i, 1));
        cent_y = round(valid_set(i, 2));
        disp_map(find(label == label(cent_y, cent_x))) = valid_set(i, 5);
    end
end

