function u_p = e2p( u_e )

u_p = [ u_e; 
        ones(1,size(u_e,2)) ];