clc
clear
close

ESP32 = serialport('COM6',115200);
configureTerminator(ESP32,'CR/LF')

expr(1)="(?<=time (s): )(?<float>[+-]?([0-9]*[.])?[0-9]+)";
expr(2)="(?<=cart_pos (cm): )(?<float>[+-]?([0-9]*[.])?[0-9]+)";
expr(3)="(?<=pend_pos (deg): )(?<float>[+-]?([0-9]*[.])?[0-9]+)";
expr(4)="(?<=cart_vel (cm/s): )(?<float>[+-]?([0-9]*[.])?[0-9]+)";
expr(5)="(?<=pend_vel (deg/s): )(?<float>[+-]?([0-9]*[.])?[0-9]+)";
expr(6)="(?<=ticks: )(?<float>[+-]?([0-9]*[.])?[0-9]+)";

i=1;
t=0;
j=1;
match=NaN(1,6);
h=plot(NaN,NaN,'r');    % Open plot object to speed up plotting speed
xlabel('time (s)');
ylabel('cart\_pos (cm)');

while true
    serial_str(i)=readline(ESP32);
    check_empty=regexp(serial_str(i),expr(1),'match');
    if ~isempty(check_empty)
        for ii=1:length(expr)
            match(ii) = regexp(serial_str(i),expr(ii),'match');
        end
        for ii=1:length(expr)
            data(j,ii)=str2double(match(ii));
        end
        % t(i)=str2double(match(1));
        % cart_pos(i)=str2double(match(2));
        % cart_vel(i)=str2double(match(3));
        set(h, 'XData', data(1:j,1),'YData',data(1:j,2));
        drawnow
        j=j+1;
    end

    i=i+1;

end

clear ESP32; % Close serial port object
