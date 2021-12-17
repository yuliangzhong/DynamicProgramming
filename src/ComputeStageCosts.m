function G = ComputeStageCosts(stateSpace, map)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

global GAMMA R P_WIND Nc
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K
global TERMINAL_STATE_INDEX

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
G=zeros(K,5);
for i=1:K
    if i==TERMINAL_STATE_INDEX
        G(i,:)=0;
        continue
    end
    for action=1:5
        current_m=stateSpace(i,1);
        current_n=stateSpace(i,2);
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
        %if action is not allowed, set cost to inf
        allowed=check_state(next_m,next_n,map);
        if not(allowed)
            G(i,action)=inf;
            continue;
        end
        %compute wind crash prob
        %go north
        wind_crash_prob_north=0;
        next_m1=next_m;
        next_n1=next_n+1;
        %check if the drone will crash
        allowed1=check_state(next_m1,next_n1,map);
        if not(allowed1)        
            wind_crash_prob_north=1;
        end
        %use position after wind to compute hit_crash prob
        hit_crash_prob_north=multi_hit_crash(next_m1,next_n1,map);
        %overall crash prob if moved to north by wind
        crash_prob_north=wind_crash_prob_north+(1-wind_crash_prob_north)*hit_crash_prob_north;
        
        %go south
        wind_crash_prob_south=0;
        next_m1=next_m;
        next_n1=next_n-1;
        %check if the drone will crash
        allowed1=check_state(next_m1,next_n1,map);
        if not(allowed1)        
            wind_crash_prob_south=1;
        end
        %use position after wind to compute hit_crash prob
        hit_crash_prob_south=multi_hit_crash(next_m1,next_n1,map);
        %overall crash prob if moved to south by wind
        crash_prob_south=wind_crash_prob_south+(1-wind_crash_prob_south)*hit_crash_prob_south;
        
        %go east
        wind_crash_prob_east=0;
        next_m1=next_m+1;
        next_n1=next_n;
        %check if the drone will crash
        allowed1=check_state(next_m1,next_n1,map);
        if not(allowed1)        
            wind_crash_prob_east=1;
        end
        %use position after wind to compute hit_crash prob
        hit_crash_prob_east=multi_hit_crash(next_m1,next_n1,map);
        %overall crash prob if moved to east by wind
        crash_prob_east=wind_crash_prob_east+(1-wind_crash_prob_east)*hit_crash_prob_east;
        
        %go west
        wind_crash_prob_west=0;
        next_m1=next_m-1;
        next_n1=next_n;
        %check if the drone will crash
        allowed1=check_state(next_m1,next_n1,map);
        if not(allowed1)        
            wind_crash_prob_west=1;
        end
        %use position after wind to compute hit_crash prob
        hit_crash_prob_west=multi_hit_crash(next_m1,next_n1,map);
        %overall crash prob if moved to west by wind
        crash_prob_west=wind_crash_prob_west+(1-wind_crash_prob_west)*hit_crash_prob_west;
        
        crash_prob_nowind=multi_hit_crash(next_m,next_n,map);
        
        crash_prob=(1-P_WIND)*crash_prob_nowind+P_WIND/4*(crash_prob_north+crash_prob_south+crash_prob_east+crash_prob_west);
        
        %compute estimated stage cost, 1+Nc if crashed, 1 if not
        G(i,action)=crash_prob*(Nc)+(1-crash_prob);
    end
end

end
