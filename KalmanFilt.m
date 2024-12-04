function [StateEst, ObsPred] = KalmanFilt(Obs,F,G, H,Q, R,x0,K0,span)
    % the Kalman filter to be implemented
    StateEst = zeros(size(F,1), span+1);
    ObsPred = zeros(size(H,1), span+1);

    StateEst(:,1) = x0;
    K_n = K0;
    Gamma = zeros(size(F,1), size(R,2));
    I = eye(size(F,1));

    for i = 1:span
        % Gamma update
        % Riccati for Gamma update  Pg 41
        K_np1 = F * K_n * F' + G * Q * G';
        K_n =  (I - Gamma*H)* K_np1;
        % Pg 37
        Sigma = R + H *  K_np1 * H';
        % Pg 40
        Gamma = K_np1 * H' / Sigma;


        % Kalman recursive update
        % obs prediction Pg 37
        ObsPred(:,i+1) = H * F * StateEst(:,i);
        % innovation Pg 39
        alpha = Obs(:,i+1) - H * F * StateEst(:,i);
        % Kalman update Pg 39
        StateEst(:,i+1) = F * StateEst(:,i) + Gamma * alpha;
    end
end
