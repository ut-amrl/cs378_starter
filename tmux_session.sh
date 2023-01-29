#!/bin/bash

tmux -V > /dev/null 2>&1
if [ $? -eq 127 ]
then
    echo "tmux is not installed! Cannot continue"
    exit 127
fi

session="ROS"

INSTALL_PATH=$1
if [ -z "${INSTALL_PATH}" ]
then
    INSTALL_PATH="${HOME}/"
fi

# check for /  at the end
if [ ${INSTALL_PATH: -1} != '/' ]
then
    INSTALL_PATH="${INSTALL_PATH}/"
fi

if ! [[ -d "${INSTALL_PATH}ut_automata" ]]
then
    echo "${INSTALL_PATH}ut_automata does not exist, please provide a valid path"
    exit 127
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