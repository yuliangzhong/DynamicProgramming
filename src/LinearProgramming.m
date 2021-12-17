function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%% Linear Programming
%min f'x s.t. Ax<=b
%max sum(V(i)) s.t. V(i)<=g(i,u)+sum(p(i,j,u)*V(j)) for any u,i except
%terminal state index

f=-ones(1,K-1);

inf_num=size(find(G==inf),1);%number of invalid actions
A=zeros(5*K-inf_num-5,K-1);
b=zeros(5*K-inf_num-5,1);
policy=zeros(5*K-inf_num-5,2);%record state&action of each equation
idx=1;
for i=1:K
    for action=1:5
        if i~=TERMINAL_STATE_INDEX && G(i,action)~=inf
            temp1=P(i,:,action);
            temp1(TERMINAL_STATE_INDEX)=[];
            temp2=zeros(1,K);
            temp2(1,i)=1;
            temp2(TERMINAL_STATE_INDEX)=[];
            A(idx,:)=temp2-temp1;
            b(idx)=G(i,action);
            policy(idx,1)=i;
            policy(idx,2)=action;
            idx=idx+1;
        end
    end
end

J_opt=linprog(f,A,b);

%find corresponding action
u_opt_ind=zeros(K,1);
action_idx=find(abs(A*J_opt-b)<1e-7);
for i=1:size(action_idx,1)
    state=policy(action_idx(i),1);
    action=policy(action_idx(i),2);
    u_opt_ind(state)=action;
end

J_opt=[J_opt(1:(TERMINAL_STATE_INDEX-1));0;J_opt(TERMINAL_STATE_INDEX:(K-1))];

end


