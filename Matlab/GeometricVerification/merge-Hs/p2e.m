function u_e = p2e( u_p )

u_e = bsxfun(@rdivide,u_p(1:end-1,:),u_p(end,:));