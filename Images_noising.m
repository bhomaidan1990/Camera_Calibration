%-----------------------------------------
clc
clear all
close all
%-----------------------------------------
path = 'Belal_checkboard/image_%03d.jpg'; % modify according to your path and images name and extension
numImages = 20; % modify according to number of images you want to read
%----------------------------------------
file_names = cell(1, numImages);
for n=1:numImages
  img = imread(sprintf(path,n));
  gray = rgb2gray(img);
  noised = imnoise(gray,'gaussian',0,0.02);
  imwrite(noised,sprintf('image_%02d.jpg',n));
end
%-----------------------------------------
