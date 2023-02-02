function [plt] = InitDataPlot( maxloops)
    plt.t = zeros(1, maxloops);
    
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);

    plt.q2 = zeros(7, maxloops);
    plt.q_dot2 = zeros(7, maxloops);

    plt.rhoLrg = zeros(1, maxloops);
    plt.rhoRrg = zeros(1, maxloops);
    plt.distLrg = zeros(1, maxloops);
    plt.distRrg = zeros(1, maxloops);
    plt.distLrg1 = zeros(1, maxloops);
    plt.distRrg1 = zeros(1, maxloops);
    plt.distLrg2 = zeros(1, maxloops);
    plt.distRrg2 = zeros(1, maxloops);
    plt.ajlL = zeros(7, maxloops);
    plt.ajlR = zeros(7, maxloops);
    %LAST PLOT
    plt.RDesAng = zeros(3, maxloops);
    plt.RDesLin = zeros(3, maxloops);
    plt.LDesAng = zeros(3, maxloops);
    plt.LDesLin = zeros(3, maxloops);
    plt.RActAng = zeros(3, maxloops);
    plt.RActLin = zeros(3, maxloops);
    plt.LActAng = zeros(3, maxloops);
    plt.LActlin = zeros(3, maxloops);

end

