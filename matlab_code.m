%===========================================
% clearing, closing, and warning supression
%===========================================
clc
clearvars
close all
warning('off','all')
%-------------------------------------------------------------
%%
%===========================
% 1-Prepare Calibration Images
%===========================
% Initialization
path = 'Belal_checkboard/image_%03d.jpg'; % modify according to your path and images name and extension
numImages = 20; % modify according to number of images you want to read
magnification = 26; % pixel per mm
squareSize = 26; % in millimeters

file_names = cell(1, numImages);
for n=1:numImages
  file_names{n} = sprintf(path,n);
end
%imshow(images(1,1))
% Display one of the calibration images

I = imread(file_names{1});
figure('Name','One of the Calibration Images','NumberTitle','off');
imshow(I, 'InitialMagnification', magnification);
title('One of the Calibration Images');
%-------------------------------------------------------------------------
%% 
%========================
% 2-Estimate Camera Parameters
%========================
% Detect the checkerboard corners in the images.
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(file_names);
% Plot the detected points
%---------------------------
file_names2 = file_names(imagesUsed);
figure('Name','detected Points','NumberTitle','off');
%for i = 1:numel(file_names2) % Uncomment_1
for i = 1:4 % Comment_1
  I = imread(file_names2{i});
  %subplot(5, 4, i); % Uncomment_2
  subplot(2, 2, i); % Comment_2
  imshow(I);
  title(['image',num2str(i)])
  hold on;
  plot(imagePoints(:,1,i),imagePoints(:,2,i),'ro');
end
%---------------------------
% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
imageSize = [size(I, 1), size(I, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints,'ImageSize', imageSize);

% Evaluate calibration accuracy.
figure('Name','Reprojection Errors','NumberTitle','off');
showReprojectionErrors(cameraParams);
title('Reprojection Errors');
%-------------------------------------------------------------------------
%%
%========================
% 3-Read the Image of Objects to Be Measured
%========================
imOrig = imread(file_names{9});
figure('Name','Input Image','NumberTitle','off');
imshow(imOrig, 'InitialMagnification', magnification);
title('Input Image');
%-------------------------------------------------------------------------
%% 
%========================
% 4-Undistort the Image
%========================
% Since the lens introduced little distortion, use 'full' output view to illustrate that
% the image was undistored. If we used the default 'same' option, it would be difficult
% to notice any difference when compared to the original image. Notice the small black borders.
[im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
figure('Name','Undistorted Image','NumberTitle','off'); 
imshow(im, 'InitialMagnification', magnification);
title('Undistorted Image');
%-------------------------------------------------------------------------
%%
%========================
% 7-Compute Extrinsics
%========================
% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);

% Adjust the imagePoints so that they are expressed in the coordinate system
% used in the original image, before it was undistorted.  This adjustment
% makes it compatible with the cameraParameters object computed for the original image.
imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints

% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);
%-------------------------------------------------------------------------
