#!/bin/bash

tmux -V > /dev/null 2>&1
if [ $? -eq 127 ]
then
    echo "tmux is not installed! Cannot continue"
    exit 127
fi

session="ROS"

INSTALL_PATH=$1
if [ $INSTALL_PATH -eq ""]
then
    INSTALL_PATH="${HOME}/"
fi

tmux new-session -d -s $session 
tmux set mouse on
window=0

tmux rename-window -t $session:$window 'roscore'
tmux send-keys -t $session:$window 'roscore' C-m

window=1
tmux new-window -t $session:$window -n 'simulator'
tmux send-keys -t $session:$window "cd ${INSTALL_PATH}/ut_automata && ./bin/simulator --localize" C-m

window=2
tmux new-window -t $session:$window -n 'websocket'
tmux send-keys -t $session:$window "cd ${INSTALL_PATH}/ut_automata && ./bin/websocket" C-m

/bin/bash