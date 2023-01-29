function [uvms] = ReceiveUdpPackets(uvms, uSensor)

condition = true;
while condition
    sensorDistance = uSensor.step();
    if (isempty(sensorDistance) == false)
        if (uvms.sensorDistance < 1 && sensorDistance == 100)
            uvms.sensorDistance = 0;
        else
            uvms.sensorDistance = double(sensorDistance);
        end
    end
    condition = (isempty(sensorDistance) == false);
end

w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw;
v_sensor = [0 0 uvms.sensorDistance]';
uvms.altitude = v_kw' * v_sensor;
    
end

