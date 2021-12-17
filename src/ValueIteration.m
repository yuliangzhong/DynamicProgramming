function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
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
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K HOVER

%% Initialization
[V_current,u_opt_ind]=min(G,[],2);
%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%J_opt(TERMINAL_STATE_INDEX)=0;

%% Standard Value Iteration
% newcost=zeros(5,1);
% V_last=zeros(K,1);
% while(any(abs(V_current-V_last)>0.00001))
%     V_last=V_current;
%     for i=1:K
%         for action=1:5
%             newcost(action)=G(i,action)+sum(P(i,:,action)'.*V_last(:));
%         end
%         [V_current(i),u_opt_ind(i)]=min(newcost);
%     end
% end

%% Value Iteration: Gauss-Seidel Update
newcost=zeros(5,1);
converged=false;
while not(converged)
    converged=true;
    for i=1:K
        for action=1:5%compute new cost-to-go for all actions
            newcost(action)=G(i,action)+P(i,:,action)*V_current(:);
        end
        [V_new,u_opt_ind(i)]=min(newcost);%find action minimize cost-to-go
        if abs(V_current(i)-V_new)>0.00001%if cost-to-go at any state gets updated, algorithm hasn't converged
            converged=false;
        end
        V_current(i)=V_new;
    end
end
J_opt=V_current;
end