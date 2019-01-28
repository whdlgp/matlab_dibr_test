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
world_coord_rt = rotate_translate(world_coord, alpha, beta, gamma, tx, ty, tz);

% Find pixel position if project to image plane with R|T applied points
[im_another_point, depth_another_point] = d_world2pixel(world_coord_rt, im1, fx, fy, ox, oy);

% dilate and erode
se = offsetstrel('ball', 7, 7);
depth_another_point_dial = imdilate(depth_another_point,se);
depth_another_point_erod = imerode(depth_another_point_dial,se);
diff_morpho = depth_another_point_dial - depth_another_point_erod;

% Test stereo show
figure(1);
subplot(1, 2, 1);
imshow(im1);
subplot(1, 2, 2);
imshow(im_another_point);

figure(2);
subplot(1, 2, 1);
show_depth(depth);
subplot(1, 2, 2);
show_depth(depth_another_point);

figure(3);
subplot(1, 4, 1);
imshow(uint8(round(rescale(depth_another_point)*255)));
subplot(1, 4, 2);
imshow(uint8(round(rescale(depth_another_point_dial)*255)));
subplot(1, 4, 3);
imshow(uint8(round(rescale(depth_another_point_erod)*255)));
subplot(1, 4, 4);
imshow(uint8(round(rescale(diff_morpho)*255)));