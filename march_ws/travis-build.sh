#!/usr/bin/env bash
read -r -d '' warning_text << EndOfMessage
#
# IMPORTANT NOTE!
#   This file is generated from trigger-dependent-build-base.sh and moved here by travis-build.sh
#   If you wish to edit it, make sure to edit the trigger-dependent-build-base.sh file,
#   so your changes are propagated to all repositories.
#
EndOfMessage


shebang=$(head -n 2 ./trigger-dependent-build-base.sh)

grep -o '^[^#]*' trigger-dependent-build-base.sh > trigger-dependent-build.sh

echo "$warning_text" | cat - trigger-dependent-build.sh > temp && mv temp trigger-dependent-build.sh
echo "$shebang" | cat - trigger-dependent-build.sh > temp && mv temp trigger-dependent-build.sh


git submodule foreach cp ../../trigger-dependent-build.sh .
git submodule foreach cp ../../../.travis.yml .

rm ./trigger-dependent-build.sh


