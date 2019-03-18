clear;
clc;
close all;

% Stereo image and disparity from 
% https://github.com/roatienza/densemapnet
% https://lmb.informatik.uni-freiburg.de/resources/datasets/SceneFlowDatasets.en.html
dir_name = 'deep_learning_dataset\';
im1 = imread([dir_name 'left.png']);

[height, width, ~] = size(im1);
fx = 15;
fy = 15;
ox = width/2;
oy = height/2;
baseline = 100; % I can't find where is baseline information for dataset, just set 100

disparityMap_gt = double(imread([dir_name 'disparity_gt.png']));
disparityMap_predict = double(imread([dir_name 'disparity_predict.png']));
depth_gt = baseline * fx ./ (disparityMap_gt);
depth_predict = baseline * fx ./ (disparityMap_predict);

% Pixel coordinate to world coordinate (mm)
world_coord_gt = d_pixel2world(depth_gt, fx, fy, ox, oy);
world_coord_predict = d_pixel2world(depth_predict, fx, fy, ox, oy);

% Rotate and translate with R|T matrix
alpha = 0;
beta = 0;
gamma = 0;
tx = -100;
ty = 0;
tz = 0;

[world_coord_rt_gt, rot_mat_gt, t_mat_gt] = rotate_translate(world_coord_gt, alpha, beta, gamma, tx, ty, tz);
[world_coord_rt_predict, rot_mat_predict, t_mat_predict] = rotate_translate(world_coord_predict, alpha, beta, gamma, tx, ty, tz);

% Find pixel position if project to image plane with R|T applied points
[im_another_point_gt, depth_another_point_gt] = d_world2pixel(world_coord_rt_gt, im1, fx, fy, ox, oy);
[im_another_point_predict, depth_another_point_predict] = d_world2pixel(world_coord_rt_predict, im1, fx, fy, ox, oy);

% dilate and erode depth map image
se1 = offsetstrel('ball', 6, 6);
se2 = offsetstrel('ball', 6, 6);
se3 = offsetstrel('ball', 2, 2);
depth_another_point_dial_gt = imdilate(depth_another_point_gt,se1);
depth_another_point_erod_gt = imerode(depth_another_point_dial_gt,se2);
depth_another_point_erod2_gt = imerode(depth_another_point_erod_gt,se3);

depth_another_point_dial_predict = imdilate(depth_another_point_predict,se1);
depth_another_point_erod_predict = imerode(depth_another_point_dial_predict,se2);
depth_another_point_erod2_predict = imerode(depth_another_point_erod_predict,se3);

% median filter depth map image
depth_another_point_median_gt = medfilt2(depth_another_point_gt, [7, 7]);
depth_another_point_median_predict = medfilt2(depth_another_point_predict, [7, 7]);

% bilateral filter depth map image
depth_another_point_bilate_gt = imbilatfilt(depth_another_point_gt);
depth_another_point_bilate_predict = imbilatfilt(depth_another_point_predict);

% Reproject to world coordinate with modified depth map image
world_coord_morpho_gt = d_pixel2world(depth_another_point_erod2_gt, fx, fy, ox, oy);
world_coord_median_gt = d_pixel2world(depth_another_point_median_gt, fx, fy, ox, oy);

world_coord_morpho_predict = d_pixel2world(depth_another_point_erod2_predict, fx, fy, ox, oy);
world_coord_median_predict = d_pixel2world(depth_another_point_median_predict, fx, fy, ox, oy);

% Rotate and translate to reverse direction(to origin)
world_coord_rt_reverse_morpho_gt = rotate_translate_reverse(world_coord_morpho_gt, rot_mat_gt, t_mat_gt);
world_coord_rt_reverse_median_gt = rotate_translate_reverse(world_coord_median_gt, rot_mat_gt, t_mat_gt);

world_coord_rt_reverse_morpho_predict = rotate_translate_reverse(world_coord_morpho_predict, rot_mat_predict, t_mat_predict);
world_coord_rt_reverse_median_predict = rotate_translate_reverse(world_coord_median_predict, rot_mat_predict, t_mat_predict);

% Render with inverse mapping
im_another_point_inverse_morpho_gt = render_inverse_mapping(world_coord_rt_reverse_morpho_gt, im1, fx, fy, ox, oy);
im_another_point_inverse_median_gt = render_inverse_mapping(world_coord_rt_reverse_median_gt, im1, fx, fy, ox, oy);

im_another_point_inverse_morpho_predict = render_inverse_mapping(world_coord_rt_reverse_morpho_predict, im1, fx, fy, ox, oy);
im_another_point_inverse_median_predict = render_inverse_mapping(world_coord_rt_reverse_median_predict, im1, fx, fy, ox, oy);

quality_check = 1;
if quality_check == 1
    % PSNR
    MSR_morpho = sum(sum(sum((im_another_point_inverse_morpho_gt - im_another_point_inverse_morpho_predict).^2)))/(width*height);
    MSR_median = sum(sum(sum((im_another_point_inverse_median_gt - im_another_point_inverse_median_predict).^2)))/(width*height);
    PSNR_morpho  = 10*log10((255^2)/MSR_morpho);
    PSNR_median = 10*log10((255^2)/MSR_median);

    % SSIM
    SSIM_morpho = ssim(im_another_point_inverse_morpho_gt, im_another_point_inverse_morpho_predict);
    SSIM_median = ssim(im_another_point_inverse_median_gt, im_another_point_inverse_median_predict);

    disp("PSNR result");
    disp(["Inverse mapping(morphorogical filter): " num2str(PSNR_morpho)]);
    disp(["Inverse mapping(median filter): " num2str(PSNR_median)]);
    disp("SSIM result");
    disp(["Inverse mapping(morphorogical filter): " num2str(SSIM_morpho)]);
    disp(["Inverse mapping(median filter): " num2str(SSIM_median)]);
end

% Test stereo show
figure(1);
imshow(im1);

figure(2);
subplot(1, 2, 1);
imshow(im_another_point_gt);
subplot(1, 2, 2);
imshow(im_another_point_predict);

figure(3);
subplot(1, 2, 1);
imshow(im_another_point_inverse_morpho_gt);
subplot(1, 2, 2);
imshow(im_another_point_inverse_morpho_predict);

figure(4);
subplot(1, 2, 1);
imshow(im_another_point_inverse_median_gt);
subplot(1, 2, 2);
imshow(im_another_point_inverse_median_predict);
