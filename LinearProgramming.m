function [ J_opt, u_opt_ind, A, b ] = LinearProgramming( P, G )
%LINEARPROGRAMMING Value iteration
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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
format long
n_states = size(G,1);
n_controls = size(G,2);
%build constrains
Nu = n_states*n_controls;
b = zeros(Nu,1);
I_special = zeros(Nu,n_states);
Piju = zeros(Nu,n_states);
index_special = 1;
%build special matrix
for k = 1:n_states
    for controls = 1:n_controls
        I_special(index_special,k) = 1;
        index_special = index_special + 1;
    end
end
%build Piju and b
index_b = 1;
for k = 1:n_states    
    for control = 1:n_controls
        b(index_b,1) = G(k,control);
        Piju(index_b,:) = P(k,:,control);
        index_b = index_b + 1;
    end
end

for k = 1:Nu
    if b(k,1) > 712
        b(k,1) = 712;
    end
end     
   
A = I_special - Piju;
%build f
f = -1*ones(n_states,1);
J_opt = linprog(f',A,b);

%find optimal policy
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

