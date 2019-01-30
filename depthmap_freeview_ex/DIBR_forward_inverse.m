clear;
clc;
close all;

% Stereo image from 
% http://vision.middlebury.edu/stereo/data/2014/
dir_name = 'Playroom-perfect\';
im1 = imread([dir_name 'im0.png']);
im2 = imread([dir_name 'im1.png']);

% Camera parameters
calib_file_txt = [dir_name 'calib.txt'];
calib_file_m = strrep(calib_file_txt,'.txt','.m');
copyfile(calib_file_txt,calib_file_m);
run(calib_file_m);

% disparity map
disparityMap_left = readpfm([dir_name 'disp0.pfm']);
disparityMap_right = readpfm([dir_name 'disp1.pfm']);
depth_left = baseline * cam0(1) ./ (disparityMap_left);
depth_right = baseline * cam1(1) ./ (disparityMap_right);

% Pixel coordinate to world coordinate (mm)
fx = cam0(1, 1);
fy = cam0(2, 2);
ox = cam0(1, 3);
oy = cam0(2, 3);
world_coord_left = d_pixel2world(depth_left, fx, fy, ox, oy);
world_coord_right = d_pixel2world(depth_right, fx, fy, ox, oy);

% Rotate and translate with R|T matrix
alpha = 0;%deg2rad(10);
beta = 0;%deg2rad(5);
gamma = 0;
tx = 0;
ty = 0;
tz = 0;

alpha_left = alpha;
beta_left = beta;
gamma_left = gamma;
tx_left = baseline/2 + tx;
ty_left = 0 + ty;
tz_left = 0 + tz;

alpha_right = alpha;
beta_right = beta;
gamma_right = gamma;
tx_right = -baseline/2 + tx;
ty_right = 0 + ty;
tz_right = 0 + tz;
[world_coord_rt_left, rot_mat_left, t_mat_left] = rotate_translate(world_coord_left, alpha_left, beta_left, gamma_left, tx_left, ty_left, tz_left);
[world_coord_rt_right, rot_mat_right, t_mat_right] = rotate_translate(world_coord_right, alpha_right, beta_right, gamma_right, tx_right, ty_right, tz_right);

% Find pixel position if project to image plane with R|T applied points
[im_another_point_left, depth_another_point_left] = d_world2pixel(world_coord_rt_left, im1, fx, fy, ox, oy);
[im_another_point_right, depth_another_point_right] = d_world2pixel(world_coord_rt_right, im2, fx, fy, ox, oy);

% dilate and erode depth map image
se1 = offsetstrel('ball', 6, 6);
se2 = offsetstrel('ball', 6, 6);
se3 = offsetstrel('ball', 2, 2);
depth_another_point_dial_left = imdilate(depth_another_point_left,se1);
depth_another_point_erod_left = imerode(depth_another_point_dial_left,se2);
depth_another_point_erod2_left = imerode(depth_another_point_erod_left,se3);

depth_another_point_dial_right = imdilate(depth_another_point_right,se1);
depth_another_point_erod_right = imerode(depth_another_point_dial_right,se2);
depth_another_point_erod2_right = imerode(depth_another_point_erod_right,se3);

% median filter depth map image
depth_another_point_median_left = medfilt2(depth_another_point_left, [7, 7]);
depth_another_point_median_right = medfilt2(depth_another_point_right, [7, 7]);

% bilateral filter depth map image
depth_another_point_bilate_left = imbilatfilt(depth_another_point_left);
depth_another_point_bilate_right = imbilatfilt(depth_another_point_right);

% Reproject to world coordinate with modified depth map image
world_coord_morpho_left = d_pixel2world(depth_another_point_erod2_left, fx, fy, ox, oy);
world_coord_median_left = d_pixel2world(depth_another_point_median_left, fx, fy, ox, oy);

world_coord_morpho_right = d_pixel2world(depth_another_point_erod2_right, fx, fy, ox, oy);
world_coord_median_right = d_pixel2world(depth_another_point_median_right, fx, fy, ox, oy);

% Rotate and translate to reverse direction(to origin)
world_coord_rt_reverse_morpho_left = rotate_translate_reverse(world_coord_morpho_left, rot_mat_left, t_mat_left);
world_coord_rt_reverse_median_left = rotate_translate_reverse(world_coord_median_left, rot_mat_left, t_mat_left);

world_coord_rt_reverse_morpho_right = rotate_translate_reverse(world_coord_morpho_right, rot_mat_right, t_mat_right);
world_coord_rt_reverse_median_right = rotate_translate_reverse(world_coord_median_right, rot_mat_right, t_mat_right);

% Render with inverse mapping
im_another_point_inverse_morpho_left = render_inverse_mapping(world_coord_rt_reverse_morpho_left, im1, fx, fy, ox, oy);
im_another_point_inverse_median_left = render_inverse_mapping(world_coord_rt_reverse_median_left, im1, fx, fy, ox, oy);

im_another_point_inverse_morpho_right = render_inverse_mapping(world_coord_rt_reverse_morpho_right, im2, fx, fy, ox, oy);
im_another_point_inverse_median_right = render_inverse_mapping(world_coord_rt_reverse_median_right, im2, fx, fy, ox, oy);

% Find occluded area
occ_morpho_left = depth_another_point_erod2_left <= 10;
occ_morpho_right = depth_another_point_erod2_right <= 10;
occ_median_left = depth_another_point_median_left <= 10;
occ_median_right = depth_another_point_median_right <= 10;

% alpha blending
t_l = [0, 0, 0]';
t_r = [baseline, 0, 0]';
t_virtue = [tx_left, ty_left, tz_left]';
alpha = norm(t_virtue - t_l)/(norm(t_virtue - t_l)+norm(t_virtue - t_r));

blended_image_morpho = alpha_blending(im_another_point_inverse_morpho_left...
                               , im_another_point_inverse_morpho_right...
                               , occ_morpho_left...
                               , occ_morpho_right...
                               , alpha);
blended_image_median = alpha_blending(im_another_point_inverse_median_left...
                               , im_another_point_inverse_median_right...
                               , occ_median_left...
                               , occ_median_right...
                               , alpha);

quality_check = 0;
if quality_check == 1
    % PSNR
    MSR_forward = sum(sum(sum((im2 - im_another_point_left).^2)))/(width*height);
    MSR_morpho = sum(sum(sum((im2 - im_another_point_inverse_morpho_left).^2)))/(width*height);
    MSR_median = sum(sum(sum((im2 - im_another_point_inverse_median_left).^2)))/(width*height);
    PSNR_forward = 10*log10((255^2)/MSR_forward);
    PSNR_morpho  = 10*log10((255^2)/MSR_morpho);
    PSNR_median = 10*log10((255^2)/MSR_median);

    % SSIM
    SSIM_forward = ssim(im_another_point_left, im2);
    SSIM_morpho = ssim(im_another_point_inverse_morpho_left, im2);
    SSIM_median = ssim(im_another_point_inverse_median_left, im2);

    disp("PSNR result");
    disp(["Forward mapping: " num2str(PSNR_forward)]);
    disp(["Inverse mapping(morphorogical filter): " num2str(PSNR_morpho)]);
    disp(["Inverse mapping(median filter): " num2str(PSNR_median)]);
    disp("SSIM result");
    disp(["Forward mapping: " num2str(SSIM_forward)]);
    disp(["Inverse mapping(morphorogical filter): " num2str(SSIM_morpho)]);
    disp(["Inverse mapping(median filter): " num2str(SSIM_median)]);
end

% Test stereo show
figure(1);
subplot(1, 2, 1);
imshow(im1);
subplot(1, 2, 2);
imshow(im2);

figure(2);
subplot(1, 2, 1);
imshow(im_another_point_left);
subplot(1, 2, 2);
imshow(im_another_point_right);

figure(3);
subplot(1, 2, 1);
imshow(im_another_point_inverse_morpho_left);
subplot(1, 2, 2);
imshow(im_another_point_inverse_morpho_right);

figure(4);
subplot(1, 2, 1);
imshow(im_another_point_inverse_median_left);
subplot(1, 2, 2);
imshow(im_another_point_inverse_median_right);

figure(5);
subplot(1, 2, 1);
show_depth(depth_left);
subplot(1, 2, 2);
show_depth(depth_right);

figure(6);
subplot(1, 2, 1);
show_depth(depth_another_point_left);
subplot(1, 2, 2);
show_depth(depth_another_point_right);

figure(7);
subplot(1, 2, 1);
show_depth(depth_another_point_erod2_left);
subplot(1, 2, 2);
show_depth(depth_another_point_erod2_right);

figure(8);
subplot(1, 2, 1);
show_depth(depth_another_point_median_left);
subplot(1, 2, 2);
show_depth(depth_another_point_median_right);

figure(9);
subplot(1, 2, 1);
imshow(uint8(round(rescale(single(depth_left))*255)));
subplot(1, 2, 2);
imshow(uint8(round(rescale(single(depth_right))*255)));

figure(10);
subplot(1, 2, 1);
imshow(uint8(round(rescale(depth_another_point_left)*255)));
subplot(1, 2, 2);
imshow(uint8(round(rescale(depth_another_point_right)*255)));

figure(11);
subplot(1, 2, 1);
imshow(uint8(round(rescale(depth_another_point_erod2_left)*255)));
subplot(1, 2, 2);
imshow(uint8(round(rescale(depth_another_point_erod2_right)*255)));

figure(12);
subplot(1, 2, 1);
imshow(uint8(round(rescale(depth_another_point_median_left)*255)));
subplot(1, 2, 2);
imshow(uint8(round(rescale(depth_another_point_median_right)*255)));

figure(13);
subplot(1, 2, 1);
imshow(occ_morpho_left);
subplot(1, 2, 2);
imshow(occ_morpho_right);

figure(14);
subplot(1, 2, 1);
imshow(occ_median_left);
subplot(1, 2, 2);
imshow(occ_median_right);

figure(15)
subplot(1, 2, 1);
imshow(blended_image_morpho);
subplot(1, 2, 2);
imshow(blended_image_median);
