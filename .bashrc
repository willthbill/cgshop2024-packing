# If not running interactively, don't do anything
[[ $- != *i* ]] && return

# IMPORTANT
export PATH="/app/venv/bin:$PATH"

### misc ###
set -o vi
ulimit -s 50000 # larger stack for python

### exports ###
# export LANGUAGE=en_US.UTF-8
# export LANG=en_US.UTF-8
# export LC_ALL="en_US.UTF-8"
export COLOR_NC='\e[0m' # No Color
export COLOR_BLACK='\e[0;30m'
export COLOR_GRAY='\e[1;30m'
export COLOR_RED='\e[0;31m'
export COLOR_LIGHT_RED='\e[1;31m'
export COLOR_GREEN='\e[0;32m'
export COLOR_LIGHT_GREEN='\e[1;32m'
export COLOR_BROWN='\e[0;33m'
export COLOR_YELLOW='\e[1;33m'
export COLOR_BLUE='\e[0;34m'
export COLOR_LIGHT_BLUE='\e[1;34m'
export COLOR_PURPLE='\e[0;35m'
export COLOR_LIGHT_PURPLE='\e[1;35m'
export COLOR_CYAN='\e[0;36m'
export COLOR_LIGHT_CYAN='\e[1;36m'
export COLOR_LIGHT_GRAY='\e[0;37m'
export COLOR_WHITE='\e[1;37m'

### history ###
HISTCONTROL=ignoreboth # dont have duplicate lines in history
HISTSIZE=1000
HISTFILESIZE=2000
shopt -s histappend
PROMPT_COMMAND="${PROMPT_COMMAND:+$PROMPT_COMMAND$'\n'}history -a;"
if [ -d /usr/share/fzf ]; then
    export FZF_DEFAULT_COMMAND='rg --files --hidden -g "!.git" '
    export FZF_CTRL_T_COMMAND="$FZF_DEFAULT_COMMAND"
    export FZF_ALT_C_COMMAND="find -L . -mindepth 1 \( -path '*/\*' -o -fstype 'sysfs' -o -fstype 'devfs' -o -fstype 'devtmpfs' -o -fstype 'proc' \) -prune -o -type d -print 2> /dev/null | cut -b3-"
    source /usr/share/fzf/key-bindings.bash
    source /usr/share/fzf/completion.bash
else
    echo "WARNING: fzf not installed, no smart reverse search etc."
fi

### shopts ###
shopt -s checkwinsize
shopt -s autocd

### aliases ###
alias tree='tree -C'
alias ls='ls --color=auto'
alias la='ls --color=auto -A'
alias l='ls --color=auto -lhA'
alias big='du -sh * | sort -h'
alias grep='grep --color=auto'
alias fgrep='fgrep --color=auto'
alias egrep='egrep --color=auto'
alias grex='grex -c'
alias mv='mv -i'
alias rm='rm -i'
alias git='youre in docker'

### prompt ###
export PS1="\[\033[32m\]\u@\h\[\033[00m\]:\[\033[34m\]\w\[\033[33m\](\$?)\[\033[31m\]$ \[\033[00m\]"
# tmux fix
export PROMPT_COMMAND="history -a; $PROMPT_COMMAND"

### source ###
source /usr/share/doc/pkgfile/command-not-found.bash

eval "$(direnv hook bash)" # direnv

### initial message ###
printf "\033[92mdocker .bashrc sourced successfully\033[0m \n"

