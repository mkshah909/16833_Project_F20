filenameSuffix = '12141834';
numPoints = 75;

mapAndTrajFilename = ['Figures/mapAndTraj', filenameSuffix, '.avi'];
outputVideo = VideoWriter(mapAndTrajFilename);
outputVideo.FrameRate = 5;
open(outputVideo);
for i = 2:numPoints
    imFilename = ['Figures/run', filenameSuffix, '/mapAndTraj', num2str(i), '.jpg'];
    img = imread(imFilename);
    writeVideo(outputVideo, img);
end
close(outputVideo);

simGenFilename = ['Figures/simGen', filenameSuffix, '.avi'];
outputVideo = VideoWriter(simGenFilename);
outputVideo.FrameRate = 5;
open(outputVideo);
for i = 2:numPoints
    imFilename = ['Figures/sim', filenameSuffix, '/gridAndCast', num2str(i), '.jpg'];
    img = imread(imFilename);
    writeVideo(outputVideo, img);
end
close(outputVideo);