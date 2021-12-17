function [ J_opt, u_opt_ind ] = PolicyIteration(P, G)
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

    function J_current=policy_evaluation(u_opt_ind)
        %solve linear functions in form AJ=B
        A_I=eye(K-1);
        A_P=zeros(K,K);
        for idx=1:K
            A_P(idx,:)=P(idx,:,u_opt_ind(idx));
        end
        A_P(TERMINAL_STATE_INDEX,:)=[];
        A_P(:,TERMINAL_STATE_INDEX)=[];
        A=A_I-A_P;
        
        if rcond(A)>1e-15 
            %disp('Using inverse matrix');
            b=zeros(K,1);
            for idx=1:K
                b(idx)=G(idx,u_opt_ind(idx));
            end
            b(TERMINAL_STATE_INDEX)=[];

            J_current=A\b;

            J_current=[J_current(1:(TERMINAL_STATE_INDEX-1));0;J_current(TERMINAL_STATE_INDEX:(K-1))];
        else
            %Apply fixed point iteration to avoid close to singular problem
            %disp('Using fixed point iteration');
            %disp(rcond(A))
            V_current=zeros(K,1);
            converged=false;
            iteration=0;
            while not(converged) && iteration<1000
                iteration=1+iteration;
                converged=true;
                V_last=V_current;
                for idx=1:K
                    V_current(idx)=G(idx,u_opt_ind(idx))+P(idx,:,u_opt_ind(idx))*V_last(:);
                end
                if abs(V_current(idx)-V_last(idx))>0.00001%if cost-to-go at any state gets updated, algorithm hasn't converged
                    converged=false;
                end
            end
            J_current=V_current;
        end
    end
%% Initialization
%[J_current,u_opt_ind]=min(G,[],2);
J_current=zeros(K,1);
u_opt_ind=5*ones(K,1);
%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%% Policy Iteration
newcost=zeros(5,1);
terminated=false;
while not(terminated)
    terminated=true;
    %policy evaluation
    J_current=policy_evaluation(u_opt_ind);
    %policy improvement
    for i=1:K
        for action=1:5
            newcost(action)=G(i,action)+P(i,:,action)*J_current(:);
        end
        [min_value,new_u]=min(newcost);
        if u_opt_ind(i)~=new_u && abs(min_value-J_current(i))>0.00001
            terminated=false;
            u_opt_ind(i)=new_u;
        end
    end
end
J_opt=J_current;
end
