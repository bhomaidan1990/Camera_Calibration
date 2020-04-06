%============================================================
clc
clear all
close all
warning('off','all')
%-------------------------------------------------------------
%% 1-Prepare Calibration Images
path = 'Belal_checkboard/image_%03d.jpg'; % modify according to your path
numImages = 20; % modify according to number of images you want to read

files = cell(1, numImages);
for n=1:numImages
  files{n} = sprintf(path,n);
end
%imshow(images(1,1))
% Display one of the calibration images
magnification = 26;
I = imread(files{1});
figure('Name','One of the Calibration Images','NumberTitle','off');
imshow(I, 'InitialMagnification', magnification);
title('One of the Calibration Images');
%-------------------------------------------------------------------------
%% 2-Estimate Camera Parameters
% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);

% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
squareSize = 26*26; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
imageSize = [size(I, 1), size(I, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
                                     'ImageSize', imageSize);

% Evaluate calibration accuracy.
figure('Name','Reprojection Errors','NumberTitle','off');
showReprojectionErrors(cameraParams);
title('Reprojection Errors');
%-------------------------------------------------------------------------

%% 3-Read the Image of Objects to Be Measured
imOrig = imread(files{9});
figure('Name','Input Image','NumberTitle','off');
imshow(imOrig, 'InitialMagnification', magnification);
title('Input Image');
%-------------------------------------------------------------------------

%% 4-Undistort the Image
% Since the lens introduced little distortion, use 'full' output view to illustrate that
% the image was undistored. If we used the default 'same' option, it would be difficult
% to notice any difference when compared to the original image. Notice the small black borders.
[im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
figure('Name','Undistorted Image','NumberTitle','off'); 
imshow(im, 'InitialMagnification', magnification);
title('Undistorted Image');
%-------------------------------------------------------------------------

% %% 5-Segment Coins
% % Convert the image to the HSV color space.
% imHSV = rgb2hsv(im);
% 
% % Get the saturation channel.
% saturation = imHSV(:, :, 2);
% 
% % Threshold the image
% t = graythresh(saturation);
% imCoin = (saturation > t);
% 
% figure('Name','Segmented Coins','NumberTitle','off');
% imshow(imCoin, 'InitialMagnification', magnification);
% title('Segmented Coins');
% %-------------------------------------------------------------------------
% 
% %% 6-Detect Coins
% % Find connected components.
% blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
%     'CentroidOutputPort', false,...
%     'BoundingBoxOutputPort', true,...
%     'MinimumBlobArea', 200, 'ExcludeBorderBlobs', true);
% [areas, boxes] = step(blobAnalysis, imCoin);
% 
% % Sort connected components in descending order by area
% [~, idx] = sort(areas, 'Descend');
% 
% % Get the two largest components.
% boxes = double(boxes(idx(1:2), :));
% 
% % Reduce the size of the image for display.
% scale = magnification / 100;
% imDetectedCoins = imresize(im, scale);
% 
% % Insert labels for the coins.
% imDetectedCoins = insertObjectAnnotation(imDetectedCoins, 'rectangle', ...
%     scale * boxes, 'penny');
% figure('Name','Detected Coins','NumberTitle','off'); 
% imshow(imDetectedCoins);
% title('Detected Coins');
% %-------------------------------------------------------------------------

%% 7-Compute Extrinsics
% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);

% Adjust the imagePoints so that they are expressed in the coordinate system
% used in the original image, before it was undistorted.  This adjustment
% makes it compatible with the cameraParameters object computed for the original image.
imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints

% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);
%-------------------------------------------------------------------------

% %% 8-a Measure the first coin
% % Adjust upper left corners of bounding boxes for coordinate system shift 
% % caused by undistortImage with output view of 'full'. This would not be
% % needed if the output was 'same'. The adjustment makes the points compatible
% % with the cameraParameters of the original image.
% boxes = boxes + [newOrigin, 0, 0]; % zero padding is added for widht and height
% 
% % Get the top-left and the top-right corners.
% box1 = double(boxes(1, :));
% imagePoints1 = [box1(1:2); ...
%                 box1(1) + box1(3), box1(2)];
% 
% % Get the world coordinates of the corners            
% worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);
% 
% % Compute the diameter of the coin in millimeters.
% d = worldPoints1(2, :) - worldPoints1(1, :);
% diameterInMillimeters = hypot(d(1), d(2));
% fprintf('Measured diameter of one penny = %0.2f mm\n', diameterInMillimeters);
% %-------------------------------------------------------------------------
% % 8-b Measure the second coin
% %=========================================================================
% % Get the top-left and the top-right corners.
% box2 = double(boxes(2, :));
% imagePoints2 = [box2(1:2); ...
%                 box2(1) + box2(3), box2(2)];
% 
% % Apply the inverse transformation from image to world            
% worldPoints2 = pointsToWorld(cameraParams, R, t, imagePoints2);            
% 
% % Compute the diameter of the coin in millimeters.
% d = worldPoints2(2, :) - worldPoints2(1, :);
% diameterInMillimeters = hypot(d(1), d(2));
% fprintf('Measured diameter of the other penny = %0.2f mm\n', diameterInMillimeters);
% %------------------------------------------------------------------------
% 
% %% 9-Measure the Distance to The First Coin
% % Compute the center of the first coin in the image.
% center1_image = box1(1:2) + box1(3:4)/2;
% 
% % Convert to world coordinates.
% center1_world  = pointsToWorld(cameraParams, R, t, center1_image);
% 
% % Remember to add the 0 z-coordinate.
% center1_world = [center1_world 0];
% 
% % Compute the distance to the camera.
% [~, cameraLocation] = extrinsicsToCameraPose(R, t);
% distanceToCamera = norm(center1_world - cameraLocation);
% fprintf('Distance from the camera to the first penny = %0.2f mm\n', ...
%     distanceToCamera);
% %-------------------------------------------------------------------------