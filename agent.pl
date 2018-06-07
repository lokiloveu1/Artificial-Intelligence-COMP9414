% Made by Loki Cheng
% COMP9414 Assignment3 Option 2
% 2018 s1
% BDI Agent


%-----!! My Part One !!--------------------------------------------------------------------------
%which binds Intentions to intents(L,[]) with L in the form [[goal(X1,Y1),[]], ... , [goal(Xn,Yn),[]]].
%Here (Xn,Yn) is the location of the monster and (X1,Y1), ... , (Xn-1,Yn-1) are places where the mininum number of stones need to be dropped in order to allow the agent to move along some path from its current location to that of the monster.

start_find(Start):-
monster(X,Y),
Start = (X,Y).

%initial_intentions(Intentions)
%initial_intentions(intents([[goal(5,3),[]],[goal(7,6),[]],[goal(8,6),[]],[goal(9,9),[]]],[])).
initial_intentions(intents(Intentions,Goal)):-
trigger(_,Goal_o),
insert_trigger_into_Goal(Goal_o,Goal),
start_find(Start),
solve(Start, Solution, _, _),
insert_Solution_into_L(Solution,Intentions),!.


insert_trigger_into_Goal([],[]).
%insert_trigger_into_Goal([goal(X,Y)],[[goal(X,Y),[]]]).
insert_trigger_into_Goal([goal(X,Y)|Tail],Goal):-
insert_trigger_into_Goal(Tail,NewGoal),
Goal = [[goal(X,Y),[]]|NewGoal].


insert_Solution_into_L([],[]).
insert_Solution_into_L([(X,Y)|Tail],L):-
insert_Solution_into_L(Tail,Lsuc),
(
not(land_or_dropped(X,Y))->
L = [[goal(X,Y),[]]|Lsuc];
land_or_dropped(X,Y)->
L = Lsuc
)
.


% This file provides code for insert_legs(), head_member() and build_path()
% used by bfsdijkstra(), ucsdijkstra(), greedy() and astar().

% insert_legs(Generated, Legs, Generated1).
% insert new legs into list of generated legs,
% by repeatedly calling insert_one_leg()

% base case: no legs to be inserted
insert_legs(Generated, [], Generated).

% Insert the first leg using insert_one_leg(); and continue.
insert_legs(Generated, [Leg|Legs], Generated2) :-
insert_one_leg(Generated, Leg, Generated1),
insert_legs(Generated1, Legs, Generated2).

% head_member(Node, List)
% check whether Node is the head of a member of List.

% base case: node is the head of first item in list.
head_member(Node,[[Node,_]|_]).

% otherwise, keep searching for node in the tail.
head_member(Node,[_|Tail]) :-
head_member(Node,Tail).

% build_path(Expanded, [[Node,Pred]], Path).

% build_path(Legs, Path)
% Construct a path from a list of legs, by joining the ones that match.

% base case: join the last two legs to form a path of one step.
build_path([[Next,Start],[Start,Start]], [Next,Start]).

% If the first two legs match, add to the front of the path.
build_path([[C,B],[B,A]|Expanded],[C,B,A|Path]) :-
build_path([[B,A]|Expanded],[B,A|Path]), ! .

% If the above rule fails, we skip the next leg in the list.
build_path([Leg,_SkipLeg|Expanded],Path) :-
build_path([Leg|Expanded],Path).


% solve(Start, Solution, G, N)
% Solution is a path (in reverse order) from start node to a goal state.
% G is the length of the path, N is the number of nodes expanded.

solve(Start, Solution, G, N) :-
%consult(pathsearch), % insert_legs(), head_member(), build_path()
ucsdijkstra([[Start,Start,0]], [], Solution, G, 1, N),!.

% ucsdijkstra(Generated, Expanded, Solution, L, N)

% The algorithm builds a list of generated "legs" in the form
% Generated = [[Node1,Prev1,G1],[Node2,Prev2,G2],...,[Start,Start,0]]
% The path length G from the start node is stored with each leg,
% and the legs are listed in increasing order of G.
% The expanded nodes are moved to another list (G is discarded)
% Expanded = [[Node1,Prev1],[Node2,Prev2],...,[Start,Start]]
% If the next leg to be expanded reaches a goal node,
% stop searching, build the path and return it.

goal_reverse(C):-
agent_at(A,B),
C = (A,B).

ucsdijkstra([[Node,Pred,G]|_Generated], Expanded, Path, G, N, N) :-
goal_reverse(Node),
build_path([[Node,Pred]|Expanded], Path).

% Extend the leg at the head of the queue by generating the
% successors of its destination node.
% Insert these newly created legs into the list of generated nodes,
% keeping it sorted in increasing order of G; and continue searching.

ucsdijkstra([[Node,Pred,G]| Generated], Expanded, Solution, G1, L, N) :-
extend(Node, G, Expanded, NewLegs),
M is L + 1,
insert_legs(Generated, NewLegs, Generated1),
ucsdijkstra(Generated1, [[Node,Pred]|Expanded], Solution, G1, M, N).

%top node
s((X,Y),NewNode,C):-
Y=<9,
Z is Y+1,
(not(land_or_dropped(X,Z))->
C is 1;
land_or_dropped(X,Z)->
C is 0),
NewNode = (X,Z).
%below node
s((X,Y),NewNode,C):-
Y>=2,
Z is Y-1,
(not(land_or_dropped(X,Z))->
C is 1;
land_or_dropped(X,Z)->
C is 0),
NewNode = (X,Z).
%right node
s((X,Y),NewNode,C):-
X=<9,
Z is X+1,
(not(land_or_dropped(Z,Y))->
C is 1;
land_or_dropped(Z,Y)->
C is 0),
NewNode = (Z,Y).
%left node
s((X,Y),NewNode,C):-
X>=2,
Z is X-1,
(not(land_or_dropped(Z,Y))->
C is 1;
land_or_dropped(Z,Y)->
C is 0),
NewNode = (Z,Y).


% Find all successor nodes to this node, and check in each case
% that the new node has not previously been expanded.

extend(Node, G, Expanded, NewLegs) :-  % write(Node),nl, % print nodes as they are expanded
findall(
[NewNode, Node, G1],
(
s(Node, NewNode, C),
not(head_member(NewNode, Expanded)),
G1 is G + C
),
NewLegs).

% base case: insert leg into an empty list.

insert_one_leg([], Leg, [Leg]).

% If we already knew a shorter path to the same node, discard this new one.

insert_one_leg([Leg1|Generated], Leg, [Leg1|Generated]) :-
Leg  = [Node,_Pred, G ],
Leg1 = [Node,_Pred1,G1],
G >= G1, ! .

% Insert the new leg in its correct place in the list (ordered by G).

insert_one_leg([Leg1|Generated], Leg, [Leg,Leg1|Generated]) :-
Leg  = [_Node, _Pred, G ],
Leg1 = [_Node1,_Pred1,G1],
G < G1,
! .

% Search recursively for the correct place to insert.

insert_one_leg([Leg1|Generated], Leg, [Leg1|Generated1]) :-
insert_one_leg(Generated, Leg, Generated1).


%-----!! My Part Two !!--------------------------------------------------------------------------
%trigger(Percepts,Goals)
%which takes a list of percepts, each of the form stone(X,Y), and converts it into a corresponding list of goals, each of the form goal(X,Y).

trigger([],[]).

% list [Head|Tail]
trigger([stone(X, Y)|Tail],[goal(X, Y)|Goals]) :-
trigger(Tail, Goals).




%-----!! My Part Three !!--------------------------------------------------------------------------
%incorporate_goals(+Goals,+Intentions,-Intentions1)
%This procedure should take two inputs, as follows:

%a set of Goals in the form of a list [goal(X1,Y1), ... , goal(Xn,Yn)]
%the current Intentions of the agent, in the form intents(Int_drop,Int_pick) where Int_drop, Int_pick are lists of intentions in the form [goal(X,Y), Plan]
%Your procedure should return the updated Intentions of the agent after inserting the new goals into Int_pick. The new goals should be inserted into the existing list in decreasing order of the length of the shortest valid path from the agents current position.
%A valid path is one which passes through only locations (X,Y) for which land_or_dropped(X,Y) is true. More precisely, a new goal should be placed immediately before the first goal in the list whose path length is longer than that of the new goal (without reordering the current list of goals). If no such valid path exists, then the new goal should not be inserted. Note that because of repeated perception of the same event, only goals not already in the list should be inserted into the list of intentions. The Plan associated with each new goal should be the empty plan (represented as the empty list []).



% Base case, no more Goals.
incorporate_goals([], Intentions, Intentions).

% Goal is already in the Int_drop list so skip it.
incorporate_goals([Goal|Tail], intents(Int_drop,Int_pick), Intentions1) :-
is_member(Goal, Int_drop),
incorporate_goals(Tail,intents(Int_drop,Int_pick), Intentions1),!.

% Goal is already in the Int_pick list so skip it.
%incorporate_goals([Goal|Tail], intents(Int_drop,Int_pick), Intentions1) :-
%is_member(Goal, Int_pick),
%incorporate_goals(Tail,intents(Int_drop,Int_pick), Intentions1),!.

% Goal is has no valid path so skip it.
incorporate_goals([Goal|Tail], intents(Int_drop,Int_pick), Intentions1) :-
find_valid_path(Goal,_),
incorporate_goals(Tail,intents(Int_drop,Int_pick), Intentions1),!.

% We only insert if its not already in the Intentions list//  is valid // is smallest.
incorporate_goals([goal(X1,Y1)|Tail], intents(Int_drop,Int_pick), Int1) :-
not(is_member(goal(X1,Y1), Int_drop)),
not(find_valid_path(goal(X1,Y1),_)),
insert(goal(X1,Y1),Int_pick,NewInt_pick),
incorporate_goals(Tail,intents(Int_drop,NewInt_pick),Int1).

% insert(+Goal, +Intentions, -Intentions1).
%   Insert the Goal, as an Intention (i.e. [goal, plan]), into the Intentions
%   list before the Plan with a goal less than it (not greater than for
%   decending order).

insert(Goal, [Int_H|Int_old], [Int_H|Int_new]) :-
goal_reverse(C),
not(closer(Goal, Int_H, C)), !,
insert(Goal,Int_old, Int_new).

insert(Goal, Int, [[Goal, []]|Int]).

% closer(a,b,agent)
closer(goal(X1, Y1), [goal(X2, Y2)|_], (X,Y)) :-
distance((X, Y),(X1, Y1),D1),
distance((X, Y),(X2, Y2),D2),
D1 < D2.


% remove element from list
remove(List,Element,NewList):-
tinclude(not(list_memberd_truth([Element])),List,NewList).


% find valid path from agent position to goal.
find_valid_path(goal(X,Y),Intentions):-
solve((X,Y), Solution, _, _),
insert_Solution_into_L(Solution,[[Intentions,[]]|_]),!.

% is_member(+Goal, +Intentions).
%   Check whether a given Goal is in the Intentions list. Each item in the
%   Intentions list is a two member list of the format [Goal, Plan]. The Plan
%   is a list of actions.

is_member(Goal, [Head|_]) :-
member(Goal, Head).

is_member(Goal, [Head|Tail]) :-
not(member(Goal, Head)),
is_member(Goal, Tail).


% all distance in Goal.
% all_distance_list(+Goal,-all_distance_list).

all_distance_list([],L):-
L=[].

all_distance_list([goal(X,Y)|Tail],L1):-
all_distance_list(Tail,L2),
goal_reverse(C),
distance(C, (X, Y), D),
L1 = [D|L2].

is_smallest(D,ShortestD):-
D=:=ShortestD.


% add_tail(+List,+Element,-List)
% Add the given element to the end of the list, without using the "append" predicate.
add_tail([],X,[X]).
add_tail([H|T],X,[H|L]):-add_tail(T,X,L).


%-----!! My Part Four !!--------------------------------------------------------------------------
%get_action(+Intentions1,-Intentions2,-Action)
%-get_action(intents(Int_drop,Int_pick),Intentions2,Action)

%which takes the agents current Intentions in the form intents(Int_drop,Int_pick) (as described above) and computes an action to be taken by the agent as well as the updated Intentions. The agent should select an intention as follows:

%-If the agent is currently holding a stone, indicated by agent_stones(1), then the first intention [goal(X,Y), Plan] in the list Int_drop of dropping intentions is selected;
%-otherwise, if the list Int_pick of picking intentions is not empty, then its first item [goal(X,Y), Plan] is selected;
%-otherwise, no intention is selected; in this case, the agents Intentions should remain as they are, and it should stay in its current location (i.e. action is move(X,Y) if it is currently at (X,Y)).
%The file gridworld.pl includes an applicable() predicate for testing whether an action is applicable. If the first action in the selected plan is applicable, the agent selects this action and updates the plan to remove the selected action. If there is no associated plan (i.e. the plan is the empty list) or the first action in the plan for the selected intention is not applicable in the current state, the agent should construct a new plan to go from its current position to the goal location and then either pick or drop a stone at that location. The plan will be a list of move actions followed by either a pick or drop action. The agent should then select the first action in this new plan, and update the list of intentions to incorporate the new plan (minus the selected first action).

% basic case,when no intention is selected(Int_pick is empty).

get_action(intents(Int_drop,[]),intents(Int_drop,[]),move(X,Y)):-
not(agent_stones(1)),
applicable(move(X,Y)),
goal_reverse((X,Y)).

% if the list Int_pick of picking intentions is not empty, then its first item [goal(X,Y), Plan] is selected;

%find pick path,if action is applicable

get_action(intents(Int_drop,[[goal(X,Y)|_]|Tail]),intents(Int_drop,[[goal(X,Y),Plan]|Tail]),Action):-
not(agent_stones(1)),
find_plan(goal(X,Y),[H|T]),
Action = H,
Plan = T.


% If the agent is currently holding a stone(agent_stones(1))

%find drop path,if action is applicable
get_action(intents([[goal(X,Y)|_]|Tail],Int_pick),intents([[goal(X,Y),Plan]|Tail],Int_pick),Action):-
agent_stones(1),
find_plan_d(goal(X,Y),[H|T]),
Action = H,
%applicable(H),
Plan = T.

find_plan(goal(X,Y),Plan):-
solve((X,Y),[_|Solution],_,_),
list_length(Solution,N),
(
N=1->
Plan = [pick(X,Y)];
N>1->
find_path(Solution,Plan)
).

%find_path([(X,Y)],[pick(X,Y)]).
find_path([(X,Y)|Tail],Plan):-
list_length(Tail,N),
(N = 0->
Plan = [pick(X,Y)];
N>0->
find_path(Tail,NewPlan),
Plan = [move(X,Y)|NewPlan]).

find_plan_d(goal(X,Y),Plan):-
solve((X,Y),[_|Solution],_,_),
list_length(Solution,N),
(
N=1->
Plan = [drop(X,Y)];
N>1->
find_path_d(Solution,Plan)
).

%find_path_d([(X,Y)],[drop(X,Y)]).
find_path_d([(X,Y)|Tail],Plan):-
list_length(Tail,N),
(N = 0->
Plan = [drop(X,Y)];
N>0->
find_path_d(Tail,NewPlan),
Plan = [move(X,Y)|NewPlan]).


%count number of element in list.
:- use_module(library(clpfd)).
list_length([], 0).
list_length([_|Ls], N) :-
list_length(Ls, X),
X #= N - 1,
N #> 0.


%-----!! My Part Five !!--------------------------------------------------------------------------
% update_intentions(+Observation, +Intentions, -Intentions1).

%to update the agents intentions, based on observation. An at(X,Y) observation should not change the agents intentions. In the case of a picked() or dropped() observation, the agent should remove the corresponding plan from its list of intentions (since this plan has now successfully been executed).

% if Observation is move(X, Y), do nothing.
update_intentions(at(_,_),Intentions1, Intentions1).

% if Observation is pick(X, Y), delete the corresponding plan in Int_pick.
update_intentions(picked(X, Y), intents(Int_drop, Int_pick), intents(Int_drop, Int_pick1)) :-
delete_plan(picked(X, Y), Int_pick, Int_pick1).

% if Observation is drop(X, Y), delete the corresponding plan in Int_drop.
update_intentions(dropped(X, Y), intents(Int_drop, Int_pick), intents(Int_drop1, Int_pick)) :-
delete_plan(dropped(X, Y), Int_drop, Int_drop1).

% delete_plan(Observation, Int, Int1)
% delete the corresponding plan in Int.

% delete plan in Int_pick.
%delete_plan(picked(X, Y),  [[goal(X, Y)|_]], []).
delete_plan(picked(X, Y),  [[goal(X, Y)|_]|Int1], Int1).

% delete plan in Int_drop.
%delete_plan(dropped(X, Y),  [[goal(X, Y),[]]], []).
delete_plan(dropped(X, Y),  [[goal(X, Y)|_]|Int1], Int1).

% incase no match, do nothing.
% delete_plan(_, Int, Int).


