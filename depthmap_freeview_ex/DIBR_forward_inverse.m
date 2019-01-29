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

% Calculate disparity map
calc_disparity = 0;
if calc_disparity == 1
    disparityMap = disparity(rgb2gray(im1),rgb2gray(im2));
else
    disparityMap = readpfm([dir_name 'disp0.pfm']);
end
depth = baseline * cam0(1) ./ (disparityMap + doffs);

% Pixel coordinate to world coordinate (mm)
fx = cam0(1, 1);
fy = cam0(2, 2);
ox = cam0(1, 3);
oy = cam0(2, 3);
world_coord = d_pixel2world(depth, fx, fy, ox, oy);

% Rotate and translate with R|T matrix
alpha = 0;
beta = 0;%deg2rad(20);
gamma = 0;
tx = 3.427890000000000e+02;
ty = 0;
tz = 0;
[world_coord_rt, rot_mat, t_mat] = rotate_translate(world_coord, alpha, beta, gamma, tx, ty, tz);

% Find pixel position if project to image plane with R|T applied points
[im_another_point, depth_another_point] = d_world2pixel(world_coord_rt, im1, fx, fy, ox, oy);

% dilate and erode depth map image
se1 = offsetstrel('ball', 6, 6);
se2 = offsetstrel('ball', 6, 6);
se3 = offsetstrel('ball', 2, 2);
depth_another_point_dial = imdilate(depth_another_point,se1);
depth_another_point_erod = imerode(depth_another_point_dial,se2);
depth_another_point_erod2 = imerode(depth_another_point_erod,se3);
diff_morpho = depth_another_point_dial - depth_another_point_erod2;

% median filter depth map image
depth_another_point_median = medfilt2(depth_another_point, [7, 7]);

% Reproject to world coordinate with modified depth map image
world_coord_morpho = d_pixel2world(depth_another_point_erod2, fx, fy, ox, oy);
world_coord_median = d_pixel2world(depth_another_point_median, fx, fy, ox, oy);

% Rotate and translate to reverse direction(to origin)
world_coord_rt_reverse_morpho = rotate_translate_reverse(world_coord_morpho, rot_mat, t_mat);
world_coord_rt_reverse_median = rotate_translate_reverse(world_coord_median, rot_mat, t_mat);

% Render with inverse mapping
im_another_point_inverse_morpho = render_inverse_mapping(world_coord_rt_reverse_morpho, im1, fx, fy, ox, oy);
im_another_point_inverse_median = render_inverse_mapping(world_coord_rt_reverse_median, im1, fx, fy, ox, oy);

% PSNR
MSR_forward = sum(sum(sum((im2 - im_another_point).^2)))/(width*height);
MSR_morpho = sum(sum(sum((im2 - im_another_point_inverse_morpho).^2)))/(width*height);
MSR_median = sum(sum(sum((im2 - im_another_point_inverse_median).^2)))/(width*height);
PSNR_forward = 10*log10((255^2)/MSR_forward);
PSNR_morpho  = 10*log10((255^2)/MSR_morpho);
PSNR_median = 10*log10((255^2)/MSR_median);

% SSIM
SSIM_forward = ssim(im_another_point, im2);
SSIM_morpho = ssim(im_another_point_inverse_morpho, im2);
SSIM_median = ssim(im_another_point_inverse_median, im2);

disp("PSNR result");
disp(["Forward mapping: " num2str(PSNR_forward)]);
disp(["Inverse mapping(morphorogical filter): " num2str(PSNR_morpho)]);
disp(["Inverse mapping(median filter): " num2str(PSNR_median)]);
disp("SSIM result");
disp(["Forward mapping: " num2str(SSIM_forward)]);
disp(["Inverse mapping(morphorogical filter): " num2str(SSIM_morpho)]);
disp(["Inverse mapping(median filter): " num2str(SSIM_median)]);

% Test stereo show
figure(1);
subplot(2, 2, 1);
imshow(im1);
subplot(2, 2, 2);
imshow(im_another_point);
subplot(2, 2, 3);
imshow(im_another_point_inverse_morpho);
subplot(2, 2, 4);
imshow(im_another_point_inverse_median);

figure(2);
subplot(2, 2, 1);
show_depth(depth);
subplot(2, 2, 2);
show_depth(depth_another_point);
subplot(2, 2, 3);
show_depth(depth_another_point_erod2);
subplot(2, 2, 4);
show_depth(depth_another_point_median);

figure(3);
subplot(2, 2, 1);
imshow(uint8(round(rescale(depth)*255)));
subplot(2, 2, 2);
imshow(uint8(round(rescale(depth_another_point)*255)));
subplot(2, 2, 3);
imshow(uint8(round(rescale(depth_another_point_erod2)*255)));
subplot(2, 2, 4);
imshow(uint8(round(rescale(depth_another_point_median)*255)));