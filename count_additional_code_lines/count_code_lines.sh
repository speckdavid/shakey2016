#!/bin/bash
printf "\n\nPlease start this script from the count_additional_code_lines folder\n\n"
cloc --exclude-list-file=black_list_files --list-file=white_list_dirs --force-lang="LISP",pddl  --force-lang="XML",launch ../ | awk '{gsub("Lisp","PDDL"); print $0}'
