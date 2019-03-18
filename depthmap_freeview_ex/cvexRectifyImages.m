function cvexRectifyImages(file1, file2)
%cvexRectifyImages Rectify two stereo images
%  cvexRectifyImages(FILE1, FILE2) rectifies stereo images FILE1 and FILE2
%  and shows the composite of the rectifications. The rectification of
%  FILE1 is shown in red while that of FILE2 is shown in cyan. You can use
%  red-cyan stereo glasses to see the 3D effect in the composite image. If
%  the function fails to rectify the images, it will not show a figure.
%
%  Notes
%  -----
%  This is a helper function in support of examples and is subject to
%  change or removal in a future release.
%
%  See also estimateUncalibratedRectification, videorectification,
%     cvexTransformImagePair

%  Copyright  The MathWorks, Inc.
%  Date: 2012/10/24 03:14:26 $

%--------------------------------------------------------------------------
% Read Stereo Image Pair
I1 = imread(file1);
if size(I1, 3) == 3
  I1 = rgb2gray(I1);
end

I2 = imread(file2);
if size(I2, 3) == 3
  I2 = rgb2gray(I2);
end

%--------------------------------------------------------------------------
% Try to rectify the images with up to 5 iterations. If an iteration fails
% to rectify the images, the function will change the parameters to use
% more memory and computational time, and to do rectification again.
numExperiment = 0;
while numExperiment < 5
  %------------------------------------------------------------------------
  % Step 1. Set Parameters
  % Set the parameters used in the algorithm. They are adjusted based on
  % the number of experiments. The effect of a larger Q is indicated after
  % the parameter.
  numExperiment = numExperiment + 1;
  Q = numExperiment;
  metricThreshold = 2000 / Q;     % Leads to more interesting points
  matchThreshold = 5 / Q;         % Leads to more matching point pairs
  numTrials = 10000 * Q;          % Allow lower inlier ratio
  confidence = 100 - 0.01 / Q;    % Allow lower inlier ratio
  rangeThreshold = [5, 2] * Q;    % Allow larger disparity
  
  %------------------------------------------------------------------------
  % Step 2. Collect Interest Points from Each Image
  blobs1 = detectSURFFeatures(I1, 'MetricThreshold', metricThreshold);
  blobs2 = detectSURFFeatures(I2, 'MetricThreshold', metricThreshold);
  
  %------------------------------------------------------------------------
  % Step 3. Select Correspondences Between Points Based on SURF Features
  [features1, validBlobs1] = extractFeatures(I1, blobs1);
  [features2, validBlobs2] = extractFeatures(I2, blobs2);
  indexPairs = matchFeatures(features1, features2, 'Metric', 'SAD', ...
    'MatchThreshold', matchThreshold);
  
  % Retrieve locations of matched points for each image
  matchedPoints1 = validBlobs1(indexPairs(:,1),:);
  matchedPoints2 = validBlobs2(indexPairs(:,2),:);
  
  %------------------------------------------------------------------------
  % Step 4. Remove Outliers Using Epipolar Constraints
  [fMatrix, epipolarInliers, status] = estimateFundamentalMatrix(...
    matchedPoints1, matchedPoints2, 'Method', 'RANSAC', ...
    'NumTrials', numTrials, 'DistanceThreshold', 0.1, ...
    'Confidence', confidence);
  
  % If the function fails to find enough inliers or if either epipole is
  % inside the image, the images cannot be rectified. The function will
  % adjust the parameters and start another iteration.
  if status ~= 0 || isEpipoleInImage(fMatrix, size(I1)) ...
      || isEpipoleInImage(fMatrix', size(I2))
    continue;
  end
  
  inlierPoints1 = matchedPoints1(epipolarInliers, :);
  inlierPoints2 = matchedPoints2(epipolarInliers, :);
  
  %------------------------------------------------------------------------
  % Step 5. Compute Rectification Transformations
  [t1, t2] = estimateUncalibratedRectification(fMatrix, ...
    inlierPoints1.Location, inlierPoints2.Location, size(I2));
  tform1 = projective2d(t1);
  tform2 = projective2d(t2);

  %------------------------------------------------------------------------
  % Step 6. Check Quality Of Rectification And Generate Rectification
  % Composite
  matchingError = pointMatchingError(I1, tform1, inlierPoints1, ...
    I2, tform2, inlierPoints2, [7, 7], rangeThreshold);
  
  % maximumMatchingError is the maximum value of registration error.
  % Increase this parameter if the images are very different.
  maximumMatchingError = 0.5;
  if matchingError < maximumMatchingError
    [J1, J2] = rectifyStereoImages(I1, I2, tform1, tform2);
    figure, imshow(stereoAnaglyph(J1, J2));
    title('Rectified Stereo Images (Red - Left Image, Cyan - Right Image)');
    return;
  end
end

%%=========================================================================
% Function pointMatchingError returns the matching error when the images
% (I1 and I2) and the points (pts1 and pts2) are transformed by the
% projective transformations (tform1 and tform2). Parameter block specifies
% the window size for computing the error. Parameter range specifies the
% maximum distance which the corresponding points can have.
%%=========================================================================
function matchingError = pointMatchingError(I1, tform1, pts1, ...
  I2, tform2, pts2, block, range)

points1 = transformPointsForward(tform1, pts1.Location);
points2 = transformPointsForward(tform2, pts2.Location);

outView = outputView(I1, tform1, I2, tform2);
J1 = imwarp(I1, tform1, 'OutputView', outView);
J2 = imwarp(I2, tform2, 'OutputView', outView);

htm = vision.TemplateMatcher('Metric', 'Sum of squared differences',...
  'OutputValue', 'Metric matrix');
count = 0;
matchingError = 0;
for idx = 1: size(points1, 1)
  p1 = round(points1(idx, :));
  [T1, flag1] = cropImage(J1, p1, block);
  p2 = round(points2(idx, :));
  [T2, flag2] = cropImage(J2, p2, block+range);
  if flag1 && flag2
    metricMatrix = step(htm, T2, T1);
    matchingError = matchingError + min(min(metricMatrix));
    count = count + 1;
  end
end

if count > 0
  matchingError = matchingError / count;
else
  matchingError = inf;
end

%%=========================================================================
% Function cropImage returns T, the sub-image of I locating at loc with
% size of 2*block+1. The function also returns flag, which is true when T
% is completely inside I and false otherwise.
%%=========================================================================
function [T, flag] = cropImage(I, loc, block)
numRows = size(I, 1);
numCols = size(I, 2);
if loc(1) > block(1) && loc(1) <= numCols-block(1) ...
    && loc(2) > block(2) && loc(2) <= numRows-block(2)
  T = I(loc(2)-block(2): loc(2)+block(2), loc(1)-block(1): loc(1)+block(1));
  flag = true;
else
  T = zeros(2*block+1);
  flag = false;
end

%%=========================================================================
% Function outputView returns an imref2d object which specifies the output
% view.
%%=========================================================================
function outView = outputView(I1, tform1, I2, tform2)
numRows = size(I1, 1);
numCols = size(I1, 2);
inPts = [1, 1; 1, numRows; numCols, numRows; numCols, 1];
outPts(1:4,1:2) = transformPointsForward(tform1, inPts);
numRows = size(I2, 1);
numCols = size(I2, 2);
inPts = [1, 1; 1, numRows; numCols, numRows; numCols, 1];
outPts(5:8,1:2) = transformPointsForward(tform2, inPts);

xSort   = sort(outPts(:,1));
ySort   = sort(outPts(:,2));
xLim(1) = ceil(xSort(4)) - 0.5;
xLim(2) = floor(xSort(5)) + 0.5;
yLim(1) = ceil(ySort(4)) - 0.5;
yLim(2) = floor(ySort(5)) + 0.5;
width   = xLim(2) - xLim(1) - 1;
height  = yLim(2) - yLim(1) - 1;
outView = imref2d([height, width], xLim, yLim);
