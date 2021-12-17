%index(m,n,p,stateSpace)
%find the index of state(m,n,p)in the stateSpace
function idx = index(m,n,p,stateSpace)
global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX
    findflag = 0;
    for itt = 1:K
        if m == stateSpace(itt,1) && n == stateSpace(itt,2) && p == stateSpace(itt,3)
            idx = itt;
            findflag = 1;
        end
    end
    if findflag ==0
        disp('ERROR, NO SUCH STATE EXISTS');
        idx = -99;
    end
    
end