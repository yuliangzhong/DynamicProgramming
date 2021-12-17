%tempN(n,u)
% return the temp n after input u
function tmpN = tempN(m,n,u,map)
    if invalid_input(m,n,u,map) == 1
        disp('ERROR! INVALID INPUT!')
        tmpN = nan;
    else
        if u == 1
            tmpN = n+1;
        elseif u == 2
            tmpN = n-1;
        else
            tmpN = n;
        end
    end
end