#!/bin/bash

session="ROS"

tmux new-session -d -s $session 
tmux set mouse on
window=0

tmux rename-window -t $session:$window 'roscore'
tmux send-keys -t $session:$window 'roscore' C-m

window=1
tmux new-window -t $session:$window -n 'simulator'
tmux send-keys -t $session:$window 'cd ~/ut_automata && ./bin/simulator --localize' C-m

window=2
tmux new-window -t $session:$window -n 'websocket'
tmux send-keys -t $session:$window 'cd ~/ut_automata && ./bin/websocket' C-m

/bin/bash