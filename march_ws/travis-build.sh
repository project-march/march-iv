#!/usr/bin/env bash

# The warning message to prepend at the trigger-dependent-build file.
read -r -d '' warning_text << EndOfMessage
#
# IMPORTANT NOTE!
#   This file is generated from trigger-dependent-build-base.sh and moved here by travis-build.sh
#   If you wish to edit it, make sure to edit the trigger-dependent-build-base.sh file,
#   so your changes are propagated to all repositories.
#
EndOfMessage

# Preserve the original shebang.
shebang=$(head -n 2 ./trigger-dependent-build-base.sh)

# Remove the comments from the base file and store the results in the trigger-dependent-build.sh.
grep -o '^[^#]*' trigger-dependent-build-base.sh > trigger-dependent-build.sh

# Prepend the warning message and shebang.
echo "$warning_text" | cat - trigger-dependent-build.sh > temp && mv temp trigger-dependent-build.sh
echo "$shebang" | cat - trigger-dependent-build.sh > temp && mv temp trigger-dependent-build.sh

# Move the travis.yml and created file to each submodule.
git submodule foreach cp ../../trigger-dependent-build.sh .
git submodule foreach cp ../../../.travis-submodule-base.yml .
git submodule foreach mv .travis-submodule-base.yml .travis.yml

# Remove the temporary created file.
rm ./trigger-dependent-build.sh


