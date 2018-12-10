function port = CreatePiInit(remoteHost)
%port = CreatePiInit(remoteHost)
% 'port' is the tcp/ip port for commanding create
%
% This file initializes tcp/ip port for use with iRobot Create
% remoteHost is the ip address of the beagleboard
% ex. CreateBeagleInit('192.168.1.141') sets ip = '192.168.1.141'
% copy this file along with PacketType.m into MatlabToolBoxiRobotCreate
%
% The tcp/ip server must be running on the Raspberry Pi before running this
% code.
%
% If you receive the error "Unsuccessful open: Connection refused: connect"
% ensure that the server code is running properly on the Raspberry Pi
%
% An optional time delay can be added after all commands
% if your code crashes frequently.  15 ms is recommended by iRobot
%
% By: Chuck Yang, ty244, 2012
% Modified By: Alec Newport, acn55, 2018

global td
td = 0.015;

createPort = 8865;

% use TCP for control commands
port = tcpip(remoteHost, createPort);

warning off

disp('Opening connection to iRobot Create...');

fopen(port);

pause(0.5)
%% Confirm two way connumication
disp('Setting iRobot Create to Control Mode...');
% Start! and see if its alive
fwrite(port,128);
pause(.1)

% Set the Create in Full Control mode
% This code puts the robot in CONTROL(132) mode, which means does NOT stop 
% when cliff sensors or wheel drops are true; can also run while plugged 
% into charger
fwrite(port,132);
pause(.1)

% light LEDS
fwrite(port,[139 25 0 128]);

% set song
fwrite(port, [140 1 1 48 20]);
pause(0.05)

% sing it
fwrite(port, [141 1])

disp('I am alive if my two outboard lights came on')

% confirmation = (fread(ports.create,4))
pause(.1)

end