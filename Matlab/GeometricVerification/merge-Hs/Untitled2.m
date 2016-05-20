% samePoints = rand(2,2);
% u1 = [0 0; 0 1; 1 0; 1 1]';
% u2 = [0 0; 0 1; -rand(2,2)]';%rand(2,2)*1]' + rand(2,4)*0;


load('tmp.mat','res');
cams = res.cams;
pairs = res.pairs;
% idx1 = 13;
% idx2 = idx1+1;
for idx2=37%numel(cams):-2:2
idx1=idx2-1;
        
img1 = imread(cams(idx1).fn);
ng = numel(pairs(idx1,idx2).groups);
for ig1=[1:ng]
    for ig2=(ig1+1):ng
        
% H1 = eye(3);
% H2 = u2H(u1,u2);
H1 = pairs(idx1,idx2).groups(ig1).T;
H2 = pairs(idx1,idx2).groups(ig2).T;

H = inv(H2)*H1;
% singularVals = svd(H)
[V,D] = eig(inv(H'))

%%
l0 = rand(3,1);
eigvecs=[];
for ie = find(imag(diag(D))==0)'
    l=V(:,ie);
    eigvecs(1,end+1) = acos(l(3));
    eigvecs(2,end) = atan(l(2)/l(1));
    alpha=0:0.001:(pi);
    R=cell(size(alpha));
    for i=1:numel(alpha)
        R{i} = vrrotvec2mat([l;alpha(i)]);
    end
    x0 = cross(l,l0);
    x=zeros(3,numel(alpha));
    for i=1:numel(alpha)
        x(:,i) = R{i}*x0;
    end
    y=H*x;
    err=zeros(size(alpha));
    for i=1:numel(alpha)
        err(i) = dot(x(:,i),y(:,i))/(norm(x(:,i))*norm(y(:,i)));
    end
    eigvecs(3,end) = min(err);
%     err = err.^2;
%     figure;plot(alpha,err);xlim([min(alpha) max(alpha)])
    figure;plot(alpha,err);ylim([-1 1]);xlim([min(alpha) max(alpha)])
    drawnow
end
%%
% figure;
% imshow(img1);
% hold on
% a=V(:,imag(diag(D))==0);
% a = bsxfun(@rdivide,a,vlen(a));
% for ia=1:size(a,2)
% X=cross(a(:,ia),[0;1;0]);
% X=p2e(X)
% Y=cross(a(:,ia),[0;1;-size(img1,1)]);
% Y=p2e(Y)
% plot([X(1) Y(1)],[X(2) Y(2)],'g-','linewidth',5);
% end

%%
theta0 = eigvecs(1,1);
phi0 = eigvecs(2,1);
theta = (theta0-0.04):0.002:(theta0+0.04);
phi = (phi0-0.01):0.01:(phi0+0.19);
% theta=0:0.05:pi;
% phi=0:0.05:(2*pi);
alpha=0:0.05:(2*pi);
err=zeros(numel(theta),numel(phi));
for it=1:numel(theta)
    l=[sin(theta(it))*cos(phi);sin(theta(it))*sin(phi);...
        repmat(cos(theta(it)),1,numel(phi))];
    for ip=1:numel(phi)
        R=cell(size(alpha));
        for i=1:numel(alpha)
            R{i} = vrrotvec2mat([l(:,ip);alpha(i)]);
        end
        x0=cross(l(:,ip),l0);
        x=zeros(3,numel(alpha));
        for i=1:numel(alpha)
            x(:,i) = R{i}*x0;
        end
        y=H*x;
        tmpErr = zeros(1,numel(alpha));
        for i=1:numel(alpha)
            tmpErr(i) = dot(x(:,i),y(:,i))/(norm(x(:,i))*norm(y(:,i)));
        end
        tmpErr = abs(tmpErr);
        err(it,ip) = min(tmpErr(:));
    end
end
figure;surf(phi,theta,err)
xlabel('phi')
ylabel('theta');
hold on;
eigvecs(1:2,:) = mod(eigvecs(1:2,:),pi);
plot3(eigvecs(2,:),eigvecs(1,:),eigvecs(3,:),'r.','markersize',30)
zlim([-1 1])
figure;contourf(phi,theta,err)
xlabel('phi')
ylabel('theta')

% phiAll=repmat(phi,1,numel(theta));
% thetaAll=repmat(theta,numel(phi),1);
% thetaAll=thetaAll(:)';
% l=[sin(thetaAll).*cos(phiAll);sin(thetaAll).*sin(phiAll);cos(thetaAll)];
% err_=err';
% eigvectors = V(:,imag(diag(D))==0);
% eigvectors = bsxfun(@rdivide,eigvectors,vlen(eigvectors));
% figure;
% scatter3(l(1,:),l(2,:),l(3,:),20,err_(:),'filled')
% hold on
% plot3(eigvectors(1,:),eigvectors(2,:),eigvectors(3,:),'r.','markersize',40)
% axis equal
% xlabel('x');ylabel('y');zlabel('z')
% [V_,D_]=eig(H);
% eigvectors_ = V_(:,imag(diag(D_))==0);
% eigvectors_ = bsxfun(@rdivide,eigvectors_,vlen(eigvectors_));
% plot3(eigvectors_(1,:),eigvectors_(2,:),eigvectors_(3,:),'k.','markersize',40)


%%
[V_,D_] = eig(H);
x0 = rand(3,1);
eigvecs_=[];
for ie = find(imag(diag(D_))==0)'
    x=V_(:,ie);
    eigvecs_(1,end+1) = acos(x(3));
    eigvecs_(2,end) = atan(x(2)/x(1));
    alpha=0:0.001:(pi);
    R=cell(size(alpha));
    for i=1:numel(alpha)
        R{i} = vrrotvec2mat([x;alpha(i)]);
    end
    l0 = cross(x,x0);
    l=zeros(3,numel(alpha));
    for i=1:numel(alpha)
        l(:,i) = R{i}*l0;
    end
    p=inv(H')*l;
    err=zeros(size(alpha));
    for i=1:numel(alpha)
        err(i) = dot(l(:,i),p(:,i))/(norm(l(:,i))*norm(p(:,i)));
    end
%     err = err.^2;
    eigvecs_(3,end) = min(err);
    figure;plot(alpha,err);ylim([-1 1]);xlim([min(alpha) max(alpha)])
    drawnow
end

%%
% theta0 = eigvecs(1,1);
% phi0 = eigvecs(2,1);
% theta0 = 1.6;
% phi0 = 3.2;
% theta = (theta0-0.1):0.005:(theta0+0.1);
% phi = (phi0-0.2):0.01:(phi0+0.2);
theta=0:0.05:pi;
phi=0:0.05:(2*pi);
alpha=0:0.05:(2*pi);
err=zeros(numel(theta),numel(phi));
for it=1:numel(theta)
    x=[sin(theta(it))*cos(phi);sin(theta(it))*sin(phi);...
        repmat(cos(theta(it)),1,numel(phi))];
    for ip=1:numel(phi)
        R=cell(size(alpha));
        for i=1:numel(alpha)
            R{i} = vrrotvec2mat([x(:,ip);alpha(i)]);
        end
        l0=cross(x(:,ip),x0);
        l=zeros(3,numel(alpha));
        for i=1:numel(alpha)
            l(:,i) = R{i}*l0;
        end
        p=inv(H')*l;
        tmpErr = zeros(1,numel(alpha));
        for i=1:numel(alpha)
            tmpErr(i) = dot(l(:,i),p(:,i))/(norm(l(:,i))*norm(p(:,i)));
        end
%         tmpErr = abs(tmpErr);
        err(it,ip) = min(tmpErr(:));
    end
end
figure;surf(phi,theta,err)
xlabel('phi')
ylabel('theta');
hold on;
eigvecs_(1:2,:) = mod(eigvecs_(1:2,:),pi);
plot3(eigvecs_(2,:),eigvecs_(1,:),eigvecs_(3,:),'r.','markersize',30)
zlim([-1 1])
figure;contourf(phi,theta,err)
xlabel('phi')
ylabel('theta')

% phiAll=repmat(phi,1,numel(theta));
% thetaAll=repmat(theta,numel(phi),1);
% thetaAll=thetaAll(:)';
% x=[sin(thetaAll).*cos(phiAll);sin(thetaAll).*sin(phiAll);cos(thetaAll)];
% err_=err';
% eigvectors_ = V_(:,imag(diag(D_))==0);
% eigvectors_ = bsxfun(@rdivide,eigvectors_,vlen(eigvectors_));
% figure;
% scatter3(x(1,:),x(2,:),x(3,:),20,err_(:),'filled')
% hold on
% plot3(eigvectors_(1,:),eigvectors_(2,:),eigvectors_(3,:),'r.','markersize',40)
% axis equal
% xlabel('x');ylabel('y');zlabel('z')
% % [V_,D_]=eig(H);
% % eigvectors_ = V_(:,imag(diag(D_))==0);
% % eigvectors_ = bsxfun(@rdivide,eigvectors_,vlen(eigvectors_));
% % plot3(eigvectors_(1,:),eigvectors_(2,:),eigvectors_(3,:),'k.','markersize',40)

    end
end

end

%%
% theta = acos(V(3,1));
% phi0=atan(V(2,1)/V(1,1));
% find(phi>(phi0-0.1) & phi<(phi0+0.1))
% phi=0:0.01:(2*pi);
% alpha=0:0.01:(2*pi);
% l=[sin(theta)*cos(phi);sin(theta)*sin(phi);repmat(cos(theta),1,numel(phi))];
% err=zeros(numel(phi),numel(alpha));
% for ip=1:numel(phi)
%     R=cell(size(alpha));
%     for i=1:numel(alpha)
%         R{i} = vrrotvec2mat([l(:,ip);alpha(i)]);
%     end
%     x0=cross(l(:,ip),l0);
%     x=zeros(3,numel(alpha));
%     for i=1:numel(alpha)
%         x(:,i) = R{i}*x0;
%     end
%     y=H*x;
%     for i=1:numel(alpha)
%         err(ip,i) = dot(x(:,i),y(:,i))/(norm(x(:,i))*norm(y(:,i)));
%     end
% end
% figure;surf(err)
% xlabel('phi')
% ylabel('alpha')
% figure;contourf(err)
% xlabel('phi')
% ylabel('alpha')




