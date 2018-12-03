 clear all;
% Set up VLfeat Toolbox here.
% I IMPLEMENT VLFEAT STARTUP IN MATLAB: SO WHEN I OPEN MATLAB VLFEAT IS SET
% AUTOMATICLY SO I DIDN'T WRITE HERE TO STARTUP.

% TO RUN MULTIPANO(MY TEST IS 3), DEREFERECE THE RELATING PART.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                 YOUR CODE HERE                               %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Read in the images that you want to stitch together.
% Better to transform RGB images into gray scale images. vl_sift requires 'single' type.
disp('reading img');
% % p1 = single(vl_imreadgray('C:\vlfeat-0.9.21-bin\vlfeat-0.9.21\data\1.png'));
% % p2 = single(vl_imreadgray('C:\vlfeat-0.9.21-bin\vlfeat-0.9.21\data\2.png'));
% % p3 = single(vl_imreadgray('C:\vlfeat-0.9.21-bin\vlfeat-0.9.21\data\3.png'));
% % p1 = imresize(p1,0.1);
% % p2 = imresize(p2,0.1);
% % p3 = imresize(p3,0.1);



% p01= imread('C:\vlfeat-0.9.21-bin\vlfeat-0.9.21\data\4.png');
% p02= imread('C:\vlfeat-0.9.21-bin\vlfeat-0.9.21\data\5.png');
% p03= imread('C:\vlfeat-0.9.21-bin\vlfeat-0.9.21\data\6.png');

%%%%%%%%%%%%%%%%%%%%%
%  P01: picture 1   %
%  P02: picture 2   %
%%%%%%%%%%%%%%%%%%%%%
p01 = imread('C:\vlfeat-0.9.21-bin\vlfeat-0.9.21\data\2.png');
p02 = imread('C:\vlfeat-0.9.21-bin\vlfeat-0.9.21\data\3.png');
p01 = imresize(p01,0.1);
p02 = imresize(p02,0.1);
% p03 = imresize(p03,0.2);
p1 = single(rgb2gray(p01));
p2 = single(rgb2gray(p02));
% p3 = single(rgb2gray(p03));

% % p1 = single(rgb2gray(p1));
% % p2 = single(rgb2gray(p2));
% % p3 = single(rgb2gray(p3));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                 YOUR CODE HERE                               %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Obtaininig SIFT Correspondences by VLfeat tool box.
% Btw, what's the meaning of the outputs?
% keep track of feature points coordinates.
disp('Obtaininig SIFT Correspondences');
[f1,d1]=vl_sift(p1);
[f2,d2]=vl_sift(p2);
% [f3,d3]=vl_sift(p3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                 YOUR CODE HERE                               %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Here you should matching the SIFT feature between adjacent images by L2 distance. 
% Please fill the function match.m
disp('matching keypoints');
matches1 = match(d1,d2);
% matches2 = match(d2,d3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                 YOUR CODE HERE                               %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Estimating Homography using RANSAC
% Please fill the function RANSACFit.m
disp('RANSACing'); 
maxIter = 10000;
N1 = size(matches1, 1);
% N2 = size(matches2, 1);
seedSetSize1 = ceil(0.3 * N1);
% seedSetSize2 = ceil(0.28 * N2);
maxInlierError = 25;
goodFitThresh1 = floor(0.95 * N1);
% goodFitThresh2 = floor(0.92 * N2);
h1 = RANSACFit((f1(1:2,:))',(f2(1:2,:))',matches1,maxIter,seedSetSize1,maxInlierError,goodFitThresh1);
% h2 = RANSACFit((f2(1:2,:))',(f3(1:2,:))',matches2,maxIter,seedSetSize2,maxInlierError,goodFitThresh2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                 YOUR CODE HERE                               %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Creating the panorama
% use cell() to store correspondent images and transformations
disp('Generating panorama pictures...');

% IMAGES = cell(1,3);
% IMAGES{1,1}= p01;
% IMAGES{1,2}= p02;
% IMAGES{1,3}= p03;
% TRANS= cell(1,2);
% TRANS{1,1}= h1;
% TRANS{1,2}= h2;

Pano = PairStitch( p01,p02, h1, 'singlepano.jpg');
% Pano = MultipleStitch( IMAGES, TRANS, 'multiplepano.jpg' );
imshow(Pano);
% In plotMatches.m you can visualize the matching results after you feed proper data stream. Feel free to create your own visualization.
