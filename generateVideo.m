outputVideo = VideoWriter('Figures/mapAndTraj6.avi');
outputVideo.FrameRate = 5;
open(outputVideo);
for i = 2:49
    imFilename = ['Figures/run6/mapAndTraj', num2str(i), '.jpg'];
    img = imread(imFilename);
    writeVideo(outputVideo, img);
end
close(outputVideo);