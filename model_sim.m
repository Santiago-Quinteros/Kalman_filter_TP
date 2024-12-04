function [State, Obs] = model_sim(F,G, H,Q, R,x0,span)

% explain why size(G1,2) and size(H1,1) here, and why chol(Q)' to left multiply the noise
  U = chol(Q)'*randn(size(G,2),span+1);  % from 0 to span
  B = chol(R)'*randn(size(H,1),span+1);

 State(:,1) = x0;
 Obs(:,1)  = H*x0 + B(:,1);

for i = 1:span
  State(:,i+1) = F*State(:,i) + G*U(:,i+1);
  Obs(:,i+1)  = H*State(:,i+1) + B(:,i+1);
endfor
