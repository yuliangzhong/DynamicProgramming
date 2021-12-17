%function p_not_shot(m,n,map): total Pr for avoiding shooting
function pr = p_not_shot(m,n,map)
global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX
    pr = 1;
    for itm = 1:size(map,1)
        for itn = 1:size(map,2)
            if map(itm,itn) == SHOOTER
                d = abs(itm-m)+abs(itn-n);
                if d<=R
                    pr = pr * (1- GAMMA/(d+1));
                end
            end
        end
    end
end