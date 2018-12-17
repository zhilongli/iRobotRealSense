function depth_array = RealSenseDist(serPort, num_points, height);

% RealSenseDist(serPort, num_points)returns an array of floats, each 
% element of [depth_array] represents the depth of the point in meters,
% from the camera. 
% Requires:
%   - [num_points] must be >= 2 and <10, is the number of points to return from 
%   the camera, taken in regular intervals horizontally across the depth
%   image. The first element is always the leftmost point of the image (~27
%   degrees from center), and the last element is always the rightmost point
%   of the image. 
%   - [height] is the integer angle in degrees from the bottom of the
%   depth image, must be in range [1, 40] inclusive. 
% Note the minimum effective distance for the depth sensor is 0.15m, and
% the maximum effective distance is ~10 meters. 
    
%Flush Buffer    
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end
if num_points>9 || num_points<2
    disp('Invalid number of points, must be between 2 and 9 (inclusive)');
    return;
end
if height<1 || height>40
    disp('Invalid height in image, must be between 1 and 40 (inclusive)');
    return;
end
warning off
global td
data_to_send = uint8(strcat('dist  ',num2str(num_points), '  ', num2str(height)));
fwrite(serPort, data_to_send);

disp('waiting for response');
while serPort.BytesAvailable==0
    pause(0.1);
    
end
resp = fread(serPort, serPort.BytesAvailable); % Get response and convert to char array
%convert the response to string 
to_str = char(resp.');
%split by empty space and convert to array of data
cell_array = strsplit(to_str, ' ');

%disp('got data!');
%fprintf('response was: %s\n',resp); % Print response to command window
depth_array = zeros(num_points, 1);
for i=1:num_points
    depth_array(i) = str2double(cell_array(i));
end

pause(td)
return
end