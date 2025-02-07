clc
clear
close

ESP32 = serialport('COM3',115200);
configureTerminator(ESP32,'CR/LF')
expr="(?<=cm: )(?<float>[+-]?([0-9]*[.])?[0-9]+)";
expr2="(?<=time: )(?<float>[+-]?([0-9]*[.])?[0-9]+)";

i=1;
t=0;
j=1;
h=plot(NaN,NaN,'r');    % Open plot object to speed up plotting speed

while true
    
    serial_str(i)=readline(ESP32);
    match = regexp(serial_str(i),expr,'match');
    match2 = regexp(serial_str(i),expr2,'match');
    if ~isempty(match)
        cart_pos(i)=str2double(match);
        t(i)=str2double(match2);
        set(h, 'XData', t(1:j),'YData',cart_pos(1:j));
        drawnow
        j=j+1;
    end

    i=i+1;

end

clear ESP32; % Close serial port object
