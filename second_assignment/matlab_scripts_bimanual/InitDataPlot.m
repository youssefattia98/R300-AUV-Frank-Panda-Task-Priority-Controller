function [plt] = InitDataPlot( maxloops)
    plt.t = zeros(1, maxloops);
    
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);

    plt.q2 = zeros(7, maxloops);
    plt.q_dot2 = zeros(7, maxloops);
    
end

