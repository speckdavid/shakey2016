#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR
cloc --exclude-list-file=black_list_files --list-file=white_list_dirs --force-lang="LISP",pddl  --force-lang="XML",launch ../ | awk '{gsub("Lisp","PDDL"); print $0}'
