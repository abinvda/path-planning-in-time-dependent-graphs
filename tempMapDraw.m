% temp Get map 
clear;
% X = imread("Maps/Screen Shot 2.png");
% I = 256-X;
% BW = imbinarize(I, 'adaptive','ForegroundPolarity','dark','Sensitivity',0.4);
% % BW = im2bw(X, 0.999);
% 
% figure(1);
% imshowpair(X,BW,'montage')

image = imread("Maps/Screen Shot 2.png");
grayImage = rgb2gray(image);
aa = im2bw(image, 0.999);
bwImage = (grayImage) < 0.2;
aa
% figure(1);
% imshowpair(grayImage,aa,'montage')

figure(2);
grid = binaryOccupancyMap((1-aa));
show(grid)

prm = mobileRobotPRM(grid,5000);
% rng(rngState)
prm.ConnectionDistance = 1000;
show(prm)