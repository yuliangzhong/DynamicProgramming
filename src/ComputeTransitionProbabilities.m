function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

    function allowed=check_state(next_m,next_n,map)%return true if the next position isn't tree or out of boundary
        m=size(map,1);
        n=size(map,2);
        allowed=false;
        if next_m>0 && next_m<=m && next_n>0 && next_n<=n%ensure not exceed boundary
            next_map=map(next_m,next_n);
            if not(next_map==TREE)%ensure not crash to tree
                allowed=true;
            end
        end
    end

    function hit_crash_prob=hit_crash(current_m,current_n,shooter_m,shooter_n)
        d=abs(current_m-shooter_m)+abs(current_n-shooter_n);
        if d>R
            hit_crash_prob=0;
        else
            hit_crash_prob=GAMMA/(d+1);
        end
    end

    function hit_crash_prob=multi_hit_crash(current_m,current_n,map)
        [shooters_m,shooters_n]=find(map==SHOOTER);
        shooters_num=size(shooters_m,1);
        survive_prob=1;
        for j=1:shooters_num
            shooter_m=shooters_m(j);
            shooter_n=shooters_n(j);
            hit_prob=hit_crash(current_m,current_n,shooter_m,shooter_n);
            survive_prob=survive_prob*(1-hit_prob);%not being hit        
        end
        hit_crash_prob=1-survive_prob;
    end

P=zeros(K,K,5);

%% step 1:apply action
P1=zeros(K,K,5);
for i=1:K
    if i==TERMINAL_STATE_INDEX
        P1(i,i,:)=0;%don't consider terminal state as it is terminated
        continue
    end
    for action=1:5
        current_m=stateSpace(i,1);
        current_n=stateSpace(i,2);
        picked=stateSpace(i,3);
        switch action
            case NORTH
                next_m=current_m;
                next_n=current_n+1;
            case SOUTH
                next_m=current_m;
                next_n=current_n-1;
            case EAST
                next_m=current_m+1;
                next_n=current_n;
            case WEST
                next_m=current_m-1;
                next_n=current_n;
            case HOVER
                next_m=current_m;
                next_n=current_n;
        end
        %check if action is allowed
        allowed=check_state(next_m,next_n,map);
        %compute transition prob. 
        if allowed
            j=find(stateSpace(:,1)==next_m & stateSpace(:,2)==next_n & stateSpace(:,3)==picked);%next state index
            P1(i,j,action)=1;
        end
    end
end

[base_m,base_n]=find(map==BASE);
crashIndex=find(stateSpace(:,1)==base_m & stateSpace(:,2)==base_n & stateSpace(:,3)==0);
%% step2&3:wind
P23=zeros(K,K);
%if no wind
P23=P23+eye(K)*(1-P_WIND);
%if wind occurs
for i=1:K
    current_m=stateSpace(i,1);
    current_n=stateSpace(i,2);
    picked=stateSpace(i,3);
    %go north
    next_m=current_m;
    next_n=current_n+1;
    %check if the drone will crash
    allowed=check_state(next_m,next_n,map);
    if not(allowed)        
        P23(i,crashIndex)=P23(i,crashIndex)+P_WIND/4;
    else
        nextIndex=find(stateSpace(:,1)==next_m & stateSpace(:,2)==next_n & stateSpace(:,3)==picked);
        P23(i,nextIndex)=P23(i,nextIndex)+P_WIND/4;
    end
    %go south
    next_m=current_m;
    next_n=current_n-1;
    %check if the drone will crash
    allowed=check_state(next_m,next_n,map);
    if not(allowed)
        P23(i,crashIndex)=P23(i,crashIndex)+P_WIND/4;
    else
        nextIndex=find(stateSpace(:,1)==next_m & stateSpace(:,2)==next_n & stateSpace(:,3)==picked);
        P23(i,nextIndex)=P23(i,nextIndex)+P_WIND/4;
    end
    %go east
    next_m=current_m+1;
    next_n=current_n;
    %check if the drone will crash
    allowed=check_state(next_m,next_n,map);
    if not(allowed)
        P23(i,crashIndex)=P23(i,crashIndex)+P_WIND/4;
    else
        nextIndex=find(stateSpace(:,1)==next_m & stateSpace(:,2)==next_n & stateSpace(:,3)==picked);
        P23(i,nextIndex)=P23(i,nextIndex)+P_WIND/4;
    end
    %go west
    next_m=current_m-1;
    next_n=current_n;
    %check if the drone will crash
    allowed=check_state(next_m,next_n,map);
    if not(allowed)
        P23(i,crashIndex)=P23(i,crashIndex)+P_WIND/4;
    else
        nextIndex=find(stateSpace(:,1)==next_m & stateSpace(:,2)==next_n & stateSpace(:,3)==picked);
        P23(i,nextIndex)=P23(i,nextIndex)+P_WIND/4;
    end
end

%% step4 angry residence
P4=eye(K,K);
[shooters_m,shooters_n]=find(map==SHOOTER);
shooters_num=size(shooters_m,1);
for i=1:K
    if i~=crashIndex
        current_m=stateSpace(i,1);
        current_n=stateSpace(i,2);
        hit_crash_prob=multi_hit_crash(current_m,current_n,map);
        P4(i,crashIndex)=hit_crash_prob;
        P4(i,i)=1-hit_crash_prob;
    end
end

%% step5 pick up/delivery
P5=zeros(K,K);
for i=1:K
    current_m=stateSpace(i,1);
    current_n=stateSpace(i,2);
    picked=stateSpace(i,3);
    current_map=map(current_m,current_n);
    if current_map==PICK_UP && picked==0
        j=find(stateSpace(:,1)==current_m & stateSpace(:,2)==current_n & stateSpace(:,3)==1);
        P5(i,j)=1;
    else
        P5(i,i)=1;
    end

end

%% overall transition prob
for i=1:5
    P(:,:,i)=P1(:,:,i)*P23*P4*P5;
end

P(TERMINAL_STATE_INDEX,TERMINAL_STATE_INDEX,:)=1;% terminal state defination
%% validate result
total_prob=sum(P,2);
if all(abs(total_prob-1)<0.00001 | abs(total_prob)<0.00001)
    disp('Transition probablities matrix is valid.');
else
    disp('Transition probablities matrix is invalid.');
end
end
