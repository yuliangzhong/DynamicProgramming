%tempM(m,u)
% return the temp m after input u
function tmpM = tempM(m,n,u,map)
    if invalid_input(m,n,u,map) == 1
        disp('ERROR! invalid input applied!')
        tmpM = nan;
    else
        if u == 3
            tmpM = m+1;
        elseif u == 4
            tmpM = m-1;
        else
            tmpM = m;
        end
    end
end