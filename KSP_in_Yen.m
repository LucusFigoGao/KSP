clc;clear;close all
% Input edge adjacency
b=input('Input edge adjacency b=');   % n*3 matrix:[st,ed,weight]
%% main program
% Judging the adjacency
disp('Directed graph:1  Undirected graph:0');
Goal=input('Goal:');
% Build the image
number=max(b(:,2));
num=[1:number];
names=sprintfc('%g',num);
G=digraph(b(:,1),b(:,2),b(:,3),names);
plot(G,'Layout','force','EdgeLabel',G.Edges.Weight)
% Generate adjacency matrix
Graph=Dijkstra_Mat_Pre(Goal,b);
% Calculate the shortest path
StartPoint=input('Input the StartPoint:');
EndPoint=input('Input the EndPoint:');
K=input('The shortest K paths:');
[A,B]=Yen_Ath(Graph,StartPoint,EndPoint,K);
disp('Shortest k path set: ')
celldisp(A)
%% 1.celldelete function   (delete the path from the cell)
function [A,flag]=celldelete(A,A_delete)
flag=1;
n=length(A);
for i=1:n
    if length(A{i})==length(A_delete)
        if A{i}==A_delete
            A{i}=[];
        end
    end
end

for i=1:n
    id(i)=isempty(A{i});
end
A(id)=[];

if isempty(A)==1
    flag=0;
end
end
%% 2.cellplus function  (add the path into the cell)
function A=cellplus(A,A_plus)
n=length(A);
i=n;
if i==n
    A{i+1}=A_plus;
end
end
%% 3.Dijkstra algorithm function (calculate the shortest path)
function [path,cost,flag]=Dijkstra(A,St,En)  
    % A is the Adjacency matrix of Graph.
    n=length(A);  flag=1;
%% DEFINE THE SYMBOLS
    temp=St;      % We use the temp to represent the Startpoint.
    Rc(1:n)=0;    % Rc_Matrix record for each point,whether it found the shortest path,
                  % if true Rc(i)=1,else RC(i)=0.
    Rc(temp)=1;   % Here the Startpoint is marked.
    Dis(1:n)=0;   % Store the value of the shortest distance for each point.
    Hui_shuo_Path(1:n)=0;      % Backtracking path
%% Start iteration
    while (Rc(En)==0)
        ing=find(Rc==0); ed=find(Rc==1);  % ing:unfound; ed:found.
        min=inf;    %  At the start of iteration,we considers the min value to be inf.
        for i=1:length(ed)
            for j=1:length(ing)
                P=Dis(ed(i)) + A(ed(i),ing(j));
                if(P<min)
                    min=P;
                    lastpoint=ed(i); newpoint=ing(j);
                end
            end
        end
        if min==Inf % Check if there is the shortest path or not.
            flag=0;
            path=[]; cost=0;            
            break;
        end
        Dis(newpoint)=min;  % Record the shortest distance of each point
        Rc(newpoint)=1;     % Mark the new point and in next iteration the 'newpoint' belongs 
                            % to the 'ed collection'.
        Hui_shuo_Path(newpoint)=lastpoint;
    end
    if flag==0
        return 
    end
    cost=Dis(En)-Dis(St);     % calculate the shortest path cost for the path.
    I=1;  path(I)=En;
    while path(I)~=St         % Backtrack the shortest path
        path(I+1)=Hui_shuo_Path(path(I));
        I=I+1;
    end
    path=fliplr(path);        % Output the path number upside down
end
%% 4.Return_min function  (return the shortest path)
function [B_min,min_cost,B]=Return_min(Graph,B)
% calculate the cost for each Path in B and select the min_cost_path
    for i=1:length(B)
        sum_cost(i)=0;
        for j=1:length(B{i})-1
            sum_cost(i)=sum_cost(i)+Graph(B{i}(j),B{i}(j+1));
        end
    end
    id=find(sum_cost==min(sum_cost));
    min_cost=min(sum_cost);
    B_min=B{id(1)};
    
% B_min=B{id(randi(length(id),1))};
% if you find min_cost_path more than one,you should calculate each path you
% have already found and find the path which has the least points. However
% if the path you've choosen at the second time still have more than two
% path,you can choose any of them to return.

%% Delete the find_Path which is the min_cost_path in B

    id_i=1;
    if length(id)>1
        for i=1:length(B)
            if id_i<=length(id) && i==id(id_i)   
                L(i)=length(B{i});
                id_i=id_i+1;
            else
                L(i)=inf;
            end
        end

       ID=find(L==min(L));
       if length(ID)>1
           B_min=B{ID(1)};
       else 
           B_min=B{ID(1)};
       end
   end
   [B,flag]=celldelete(B,B_min);
end
%% 5.wh_exists function  (whether the path in the cell or not)
function target=wh_exists(A,A_delete)
    target=0;
    for i=1:length(A)
        if length(A{i})==length(A_delete)
            if A{i}==A_delete
                target=1;
            end
        end
    end
end
%% 6.Yen_algorithm main function  (calculate the K shortest paths)
function [A,B]=Yen_Ath(Graph,StartPoint,EndPoint,K)
	St=StartPoint;En=EndPoint;
	B={[1:length(Graph)]};  % Potential alternative paths
	%% shortest path
	[path,cost]=Dijkstra(Graph,St,En);
	A_path={path}; A_cost={cost}; A={A_path,A_cost};
	graph=Graph;
	
	%% KSP-Yen
	for k=2:K
	    for i=1:length(A_path{k-1})-1
	        spurnode=A_path{k-1}(i);
	        rootpath=A_path{k-1}(1:i);
	            for j=1:length(A_path)   
	                if length(rootpath)<=length(A_path{j})
	                    if rootpath == A_path{j}(1:i)	
	                        graph(A_path{j}(i),A_path{j}(i+1))=inf;
	                    end
	                end
	            end
	            [spurpath,spur_path_cost,flag]=Dijkstra(graph,spurnode,En);
	            if flag~=0
	                EntirePath=[rootpath,spurpath(2:end)];
	                target=wh_exists(B,EntirePath);
	                if target==0
	                    B=cellplus(B,EntirePath);
                	end            
            	end
            	graph=Graph;
     	end
    	[B_min,min_cost,B]=Return_min(Graph,B);
    	A_path=cellplus(A_path,B_min);  A_cost=cellplus(A_cost,min_cost);
    	A={A_path,A_cost};
	end
end
%% 7.Testing program  (KSP_Yen_MATLAB)