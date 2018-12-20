#!/bin/bash
#
# IMPORTANT NOTE!
#   This file is generated from trigger-dependent-build-base.sh and moved here by travis-build.sh
#   If you wish to edit it, make sure to edit the trigger-dependent-build-base.sh file,
#   so your changes are propagated to all repositories.
#
DOWNSTREAM_REPO_SLUG="project-march%2Fmarch-iv"
if [ "${TRAVIS_PULL_REQUEST}" == "false" ]; then
  exit 0
fi
body='{
 "request": {
     "message": "Build triggered by '${TRAVIS_REPO_SLUG}'",
     "branch":"develop",
     "config": {
        "merge_mode": "deep_merge",
        "env": {
            "global": {
                "DEPENDENT_BUILD": "true",
                "TRIGGER_BRANCH": "'${TRAVIS_PULL_REQUEST_BRANCH}'",
                "TRIGGER_REPO": "'${TRAVIS_REPO_SLUG}'"
            }
        }
      }
}}'
sudo apt-get install jq
build=$(curl -X POST \
 -H "Content-Type: application/json" \
 -H "Accept: application/json" \
 -H "Travis-API-Version: 3" \
 -H "Authorization: token ${AUTH_TOKEN}" \
 -d "$body" \
 https://api.travis-ci.org/repo/${DOWNSTREAM_REPO_SLUG}/requests | jq '.request.id')
get-state () {
    state=$(curl -X GET \
    -H "Content-Type: application/json" \
    -H "Accept: application/json" \
    -H "Travis-API-Version: 3" \
    -H "Authorization: token ${AUTH_TOKEN}" \
    https://api.travis-ci.org/repo/${DOWNSTREAM_REPO_SLUG}/request/$1 | jq '.builds[0].state')
    echo ${state}
}
if [ -n "${build}" ]; then
    echo "build not created correctly"
    exit 1
fi
travis_states=("created" "started" "passed" "canceled" "errored" "failed")
travis_success_states=("passed")
travis_continue_states=("created" "started")
travis_fail_states=("canceled" "errored" "failed")
sleep 10
while true; do
    state=$(get-state "$build")
    if [[ ! " ${travis_states[@]} " =~ "${build}" ]]; then
        echo "state $build is invalid"
        exit 1
    fi
    if [[ " ${travis_fail_states[@]} " =~ "${build}" ]]; then
        echo "dependent build ${state}"
        exit 1
    fi
    if [[ " ${travis_success_states[@]} " =~ "${build}" ]]; then
        echo "dependent build ${state}"
        exit 0
    fi
done
exit 1
