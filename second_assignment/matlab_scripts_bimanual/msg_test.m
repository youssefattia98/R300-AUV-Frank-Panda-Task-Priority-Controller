hudps = dsp.UDPSender('RemoteIPPort',1500);
hudps.RemoteIPAddress = '127.0.0.1';
a = [0 0 0 0 0 0]';

for i = 1:inf
    posMSG = [a ; a];
    step(hudps, posMSG);
end