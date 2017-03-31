function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Value iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
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
format long

n_states = size(G,1);
n_controls = size(G,2);
u = zeros(n_states,1);
u_old = zeros(n_states,1);

% intialization of u. pick argmin (G)
for i = 1:n_states
    [a, u_min] = min(G(i,:));
    u(i,1)=u_min;     
end

u = 5*ones(n_states,1);
it = 0;
while not ( isequal(u_old, u)) && it<100
    it = it+1;
    u_old = u;
    
    % construct g and P
    for k = 1:n_states
        g(k,1) = G(k,u_old(k));
        row_P_k = P(k,:,u_old(k));
        P_u(k,:) = row_P_k;
        
        
    end
    % end construct g and P
    
    % solve for cost J   
    aux = P_u;
    J = (eye(n_states)-P_u)\g;
    % que passa si eye(n_states)-P_u no es invertible
    % preguntar q   
    
    % actualize policy
    for k = 1:n_states
        P_u_aux = [];
        for index_c = 1:n_controls
            P_u_aux = [P_u_aux;P(k,:,index_c)];
            
        end
         
        %  G(i,:)'+P_u*J . matriu de 5 files i 1 columna per a l estat i
        % cada fila = un control diferent
        % guarda nou valor (numero) J(i) en el vector J_new
        
        [aux,u(k,1)] = min(G(k,:)'+P_u_aux*J);      
    end   
end
    J_opt = J;
    u_opt_ind =u;
end