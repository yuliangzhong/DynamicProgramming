%invalid_input(m,n,u,map)
%check the input is invalid(return 1) or valid(return 0) with input u
function bool=invalid_input(m,n,u,map)
global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX
    bool = 0;
    if u == 1
        if n == size(map,2)
            bool = 1;
        elseif map(m,n+1) == TREE
            bool = 1;
        end
    elseif u == 2
        if n == 1
            bool = 1;
        elseif map(m,n-1) == TREE
            bool = 1;
        end
    elseif u == 3
        if m == size(map,1)
            bool = 1;
        elseif map(m+1,n) == TREE
            bool = 1;
        end
    elseif u == 4
        if m == 1
            bool = 1;
        elseif map(m-1,n) == TREE
            bool = 1;
        end
    end
end