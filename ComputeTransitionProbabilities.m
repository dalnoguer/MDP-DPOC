function P = ComputeTransitionProbabilities( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace,
%   map, gate, mansion, cameras) computes the transition probabilities
%   between all states in the state space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 2)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       controlSpace:
%           A (L x 1)-matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map.
%           Positive values indicate cells that are inaccessible (e.g.
%           trees, bushes or the mansion) and negative values indicate
%           ponds or pools.
%
%   	gate:
%          	A (1 x 2)-matrix describing the position of the gate.
%
%    	mansion:
%          	A (F x 2)-matrix indicating the position of the cells of the
%           mansion.
%
%    	cameras:
%          	A (H x 3)-matrix indicating the positions and quality of the 
%           cameras.
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

% put your code here
%function to move given old_position
global p_c gamma_p pool_num_time_steps detected_additional_time_steps
function K_new_position = move(K_in_stateSpace,direction,stateSpace)
    %see if it is possible to find a consecutive positon in stateSpace in a
    %defined direction
    switch direction
        case 'n' 
            position_old = stateSpace(K_in_stateSpace, :);
            position_new = [position_old(1),position_old(2)+1];
        case 'e'
            position_old = stateSpace(K_in_stateSpace, :);
            position_new = [position_old(1)+1,position_old(2)];    
        case 's'
            position_old = stateSpace(K_in_stateSpace, :);
            position_new = [position_old(1),position_old(2)-1];
        case 'w'
            position_old = stateSpace(K_in_stateSpace, :);
            position_new = [position_old(1)-1,position_old(2)];    
        case 'p' 
            position_old = stateSpace(K_in_stateSpace, :);
            position_new = [position_old(1),position_old(2)];    
    end
    K_new_position = find(ismember(stateSpace,position_new,'rows'));
    if isempty(K_new_position)
        K_new_position = 0;
    end
end

function param = is_camera(p,cameras)
    % p : [x,y]
    % output: param 
    [number_cameras,columns] = size(cameras);
    param = 0;
    for camera_number = 1:number_cameras
        if cameras(camera_number,1) == p(1) & cameras(camera_number,2) == p(2)
            param = cameras(camera_number,3);
            break
        end
    end    
end

function param = is_mansion(p,mansion)
    % p : [x,y]
    % Boolean 
    [number_mansion,columns] = size(mansion);
    param = false;
    for mansion_number = 1:number_mansion
        if mansion(mansion_number,1) == p(1) & mansion(mansion_number,2) == p(2)
            param = true;
            break
        end
    end    
end
      
n_states = size(stateSpace,1);
Prob_get_caught_state = zeros(n_states,1);
for K = 1:n_states
    Prob_stay = 1;
    for direction = 1:4
        found_camera_obstacle = false;
        switch direction 
            case 1
                d = 'n';
                Delta = [0,1];
            case 2
                d = 'e';
                Delta = [1,0];
            case 3
                d = 's';
                Delta = [0,-1];
            case 4
                d = 'w';
                Delta = [-1,0];
        end
        distance = 1;
        K_old = K;
        while found_camera_obstacle == false            
            K_new = move(K_old,d,stateSpace);
            if K_new == 0
                position = stateSpace(K_old,1:2);
                pos_to_check = position + Delta;
                param = is_camera(pos_to_check,cameras);
                if param > 0
                    Prob_stay = Prob_stay * (1-param/distance);                    
                end
                found_camera_obstacle = true;
            else
                distance = distance + 1;
                K_old = K_new;                
            end            
        end
        Prob_get_caught_state(K) =1-Prob_stay;
    end    
end
%Probabilities of getting a good picture
P_good_pic = zeros(n_states,1);
for K = 1:n_states
    Prob = p_c;
    for direction = 1:4
        found_mansion = false;
        switch direction 
            case 1
                d = 'n';
                Delta = [0,1];
            case 2
                d = 'e';
                Delta = [1,0];
            case 3
                d = 's';
                Delta = [0,-1];
            case 4
                d = 'w';
                Delta = [-1,0];
        end
        distance = 1;
        K_old = K;
        while found_mansion == false            
            K_new = move(K_old,d,stateSpace);
            if K_new == 0
                position = stateSpace(K_old,1:2);
                pos_to_check = position + Delta;
                if is_mansion(pos_to_check,mansion)
                    Prob = [Prob, gamma_p/distance];                  
                end
                found_mansion = true;
            else
                distance = distance + 1;
                K_old = K_new;                
            end            
        end
        P_good_pic(K)=max(Prob);
    end    
end
n_control = size(controlSpace,1);
P = zeros(n_states,n_states,n_control);
K_gate = find(ismember(stateSpace,gate,'rows'));
for u_index = 1:n_control
    
    u = controlSpace(u_index);
    switch u 
        case 'n'
            index_to_P = 1;
        case 'w'
            index_to_P = 2;
        case 's'
            index_to_P = 3;
        case 'e'
            index_to_P = 4;
        case 'p'
            index_to_P = 5;                
    end
    for K = 1:n_states
        K_intended = move(K,u,stateSpace);
        if K_intended == 0
            K_intended = K;
        end
        position_intended = stateSpace(K_intended,:);
        if u~='p'
            if K_intended == K_gate
                P(K,K_gate,index_to_P) = 1;
            elseif map(position_intended(2),position_intended(1))<0
                P(K,K_intended,index_to_P) = (1-Prob_get_caught_state(K_intended))^pool_num_time_steps;
                P(K,K_gate,index_to_P) = 1-(1-Prob_get_caught_state(K_intended))^pool_num_time_steps;
            else
                P(K,K_intended,index_to_P) = 1-Prob_get_caught_state(K_intended);
                P(K,K_gate,index_to_P) = Prob_get_caught_state(K_intended);
            end  
        elseif u == 'p'
            if K_intended == K_gate
                P(K,K_gate,index_to_P) = (1-P_good_pic(K));
            else 
                P(K,K_intended,index_to_P) = (1-P_good_pic(K))*(1-Prob_get_caught_state(K));
                P(K,K_gate,index_to_P) = (1-P_good_pic(K))*Prob_get_caught_state(K);
            end
        end
    end       
end 
end