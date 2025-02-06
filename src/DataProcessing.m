clc
clear
close

ESP32 = serialport('COM3',115200);

i=1;
t=0;
tic;
j=1;
h=plot(NaN,NaN,'r');    % Open plot object to speed up plotting speed

while (t <= 5)
    
    t=toc; % log time since loop start

    check_identifier(i)=read(ESP32,1,'char');

    if check_identifier(i)==':'
        if check_identifier(i-1)=='m'
            if check_identifier(i-2)=='c'
                data(j,1) = t;
                data(j,2)=read(ESP32,1,'double');
                set(h, 'XData', data(1:j,1),'YData',data(1:j,2));
                drawnow
                j=j+1;
            end
        end
    end

    i=i+1;

end

clear ESP32; % Close serial port object
