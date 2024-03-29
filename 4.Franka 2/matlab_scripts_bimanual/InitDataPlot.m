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
    plt.rhoLrg1 = zeros(1, maxloops);
    plt.rhoRrg1 = zeros(1, maxloops);
    plt.distLrg1 = zeros(1, maxloops);
    plt.distRrg1 = zeros(1, maxloops);
    plt.ajlL = zeros(7, maxloops);
    plt.ajlR = zeros(7, maxloops);
    plt.RCoopAng = zeros(3,maxloops);
    plt.RCoopLin = zeros(3,maxloops);
    plt.LCoopAng = zeros(3,maxloops);
    plt.LCoopLin = zeros(3,maxloops);
    plt.RNonCoopAng = zeros(3,maxloops);
    plt.RNonCoopLin = zeros(3,maxloops);
    plt.LNonCoopAng = zeros(3,maxloops);
    plt.LNonCoopLin = zeros(3,maxloops);
    plt.w_dist_Leeg2 = zeros(1,maxloops);
    plt.w_dist_Reeg2 = zeros(1,maxloops);

end

