%% Learning the Kalman Filter through Simulation Examples

%% Overview
% The example shows how the Kalman Filter can be
% implemented in Octave/matlab.

% The model (Eq.1 Eq.2) is configured with a Gaussian
% process and its output Y_k connected to a Kalman Filter. To directly use this model, one
% only needs to provide model prarameters including parameters of the
% Gaussian process, which are state space matrices, A, B, C, and D, initial
% state, x0, and covariance matrices, Q and R; and similar parameters for
% the Kalman Filter, which can be in different values to mimic the model
% mismatch, plus the state covariance, P. The following examples show how
% this model can be used.
%
% The Kalman Filter can also be used as a standard model block to be
% connected with any other systems.



%%  A 2-input 2-output 4-state system with non-zero D
clear all,
close all,
 dt = 0.1;
randn('seed',11);


% Model parameters (Eq.1 & 2)
F = [0.8110   -0.0348    0.0499    0.3313
     0.0038    0.8412    0.0184    0.0399
     0.1094    0.4094    0.6319    0.1080
    -0.3186   -0.0254   -0.1446    0.8391];
G = [-0.0130    0.0024
     -0.0011    0.0100
     -0.0781    0.0009
      0.092    0.0138];
H = [0.1685   -0.9595   -0.0755   -0.3771
     0.6664    -0.0835    0.6260    -0.6609];
% process noise variance
Q=diag([0.8^2 0.5^2]);
% measurement noise variance
R= .1*eye(2);
% initial state
x0 = 4*randn(4,1);


% Kalman filter set up (We first set identical parameters as above)
% The same model
F1 =F;
G1 = G;
H1 = H;
Q1 = Q;
R1 = R;
% However, zeros initial state
x1 = x0; % zeros(4,1);
x1 = zeros(4,1)
% Initial state covariance
P1 = 4*eye(4);
% Simulation set up
% time span 400 samples
span = 400;

% explain why the Cholesky function is needed here for noise generation



% simulation
[State, Obs] = model_sim(F,G, H,Q, R,x0,span); % Eq1 & 2
F1=F+0.25*randn(size(F))
%  Kalman Filtering
[StateEst, ObsPred] = KalmanFilt(Obs,F1,G1, H1,Q1, R1,x1,P1,span);


t=0:span;
figure
%set(gcf,'Position',[100 100 600 800])
for k=1:4
    subplot(4,1,k), hold
    plot(t, State(k,:),'b','linewidth',2);

    subplot(4,1,k),
    plot(t, StateEst(k,:),'r','linewidth',2);
    legend('Actual state', 'Estimated state','Location','northeast');
    title(sprintf('state %i',k))
 end
xlabel('time, s')


figure(2)

for k=1:2
    subplot(2,1,k), hold
    plot(t,Obs(k,:),'b','linewidth',2);
    title(sprintf('Obs %i',k))
    subplot(2,1,k),
    plot(t, ObsPred(k,:),'r','linewidth',2);
    legend('Actual Observation','1-step predicted observation','Location','northeast');
 end
xlabel('time, s')
