function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space.

% put your code here
%Initialization of J
format long
n_states = size(G,1);
n_controls = size(G,2);
J = zeros(n_states,1);
for i = 1:n_states
    J(i) = min(G(i,:));
end
J_new = zeros(n_states,1);
error = 1;
epsilon = 0.000001;
%start recursive process
while error > epsilon
    
    for i = 1:n_states
        P_u = [];
        for u = 1:n_controls
            P_u = [P_u;P(i,:,u)];
        end
        %  G(i,:)'+P_u*J . matriu de 5 files i 1 columna per a l estat i
        % cada fila = un control diferent
        % guarda nou valor (numero) J(i) en el vector J_new
        J_new(i) = min(G(i,:)'+P_u*J);
    end
    error = max(abs(J-J_new));
    J = J_new; 
end
J_opt = J;
%find optimal policy
J_u_opt = zeros(n_states,1);
u_opt_ind = zeros(n_states,1);
for i = 1:n_states
    P_u = [];
    for u = 1:n_controls
        P_u = [P_u;P(i,:,u)];
    end
    J_u_opt =  G(i,:)'+P_u*J_opt;
    [M,I] = min(J_u_opt);
    u_opt_ind(i) = I;   
end
end